/*
 * CameraStream: Gazebo system that streams every camera sensor of the world as
 * H.264 over RTP/UDP (e.g. to QGroundControl), with a hardware encoder where one
 * works on this machine.
 *
 * Encoder selection ("auto"): each candidate is test-run once in a separate
 * gst-launch process with a timeout (a driver that hangs, e.g. NVENC on some
 * hybrid-graphics laptops, cannot block the simulation), the first one that
 * works is used. If it fails later at runtime the stream is rebuilt with the
 * next working candidate.
 *
 *   nvenc   NVIDIA          nvh264enc
 *   nvcuda  NVIDIA (>=1.22) nvcudah264enc
 *   vaapi   AMD / Intel     vaapih264enc (VA-API on the AMD/Intel render node,
 *                           also on laptops whose display runs on NVIDIA)
 *   va      AMD / Intel     vah264enc (GStreamer >= 1.22)
 *   qsv     Intel           qsvh264enc (GStreamer >= 1.22)
 *   x264    software        x264enc
 *
 * Parameters (plugin element in the server config), environment overrides in brackets:
 *   <udp_host>      127.0.0.1                  [CAMERA_STREAM_HOST]
 *   <udp_port>      5600: first port for cameras not in <ports>  [CAMERA_STREAM_PORT]
 *   <ports>         fixed ports by camera, "name=port[+port...],..." where name is matched
 *                   in the image topic (model or sensor name), e.g.
 *                   nose_camera=5600+5610,talon_cam=5601: several ports = same stream to each
 *   <encoders>      auto | comma list          [CAMERA_STREAM_ENCODERS]
 *   <bitrate_kbps>  8000 at 1920x1080, scaled with the image size
 *   <fps>           30   (nominal, used for the stream caps and the key frame interval)
 *   <camera_filter> regex on the image topic, default: all cameras
 *   <probe_timeout> 6    [s] per encoder test
 *
 * CAMERA_STREAM_STATS=1 prints every 5 s per camera: frames in from Gazebo (rate and
 * min/max interval), frames out of the encoder, and the time to hand a frame over.
 */

#include <gz/common/Console.hh>
#include <gz/msgs/image.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace interceptor_sim
{

namespace
{

struct Encoder {
	std::string name;
	std::vector<std::string> elements;  // factories that must exist
	std::string description;            // gst-launch syntax, %b bitrate kbps, %g key frame interval
};

const std::vector<Encoder> &allEncoders()
{
	static const std::vector<Encoder> encoders{
		{"nvenc", {"nvh264enc"}, "nvh264enc bitrate=%b rc-mode=cbr gop-size=%g zerolatency=true"},
		{"nvcuda", {"nvcudah264enc"}, "nvcudah264enc bitrate=%b rate-control=cbr gop-size=%g"},
		{"vaapi", {"vaapipostproc", "vaapih264enc"}, "vaapipostproc ! vaapih264enc rate-control=cbr bitrate=%b keyframe-period=%g"},
		{"va", {"vah264enc"}, "vah264enc rate-control=cbr bitrate=%b key-int-max=%g"},
		{"qsv", {"qsvh264enc"}, "qsvh264enc bitrate=%b gop-size=%g"},
		{"x264", {"x264enc"}, "x264enc bitrate=%b speed-preset=ultrafast tune=zerolatency key-int-max=%g threads=0"},
	};
	return encoders;
}

std::string replaceAll(std::string s, const std::string &from, const std::string &to)
{
	for (size_t p = s.find(from); p != std::string::npos; p = s.find(from, p + to.size())) {
		s.replace(p, from.size(), to);
	}

	return s;
}

std::string getenvOr(const char *name, const std::string &fallback)
{
	const char *v = std::getenv(name);
	return (v && *v) ? v : fallback;
}

/// VA-API must use the AMD/Intel GPU: on hybrid laptops the first render node may be NVIDIA's.
void selectVaapiDevice()
{
	if (std::getenv("GST_VAAPI_DRM_DEVICE")) {
		return;
	}

	namespace fs = std::filesystem;

	if (!fs::exists("/sys/class/drm")) {
		return;
	}

	for (const auto &entry : fs::directory_iterator("/sys/class/drm")) {
		const std::string node = entry.path().filename();

		if (node.rfind("renderD", 0) != 0) {
			continue;
		}

		std::error_code ec;
		const std::string driver = fs::read_symlink(entry.path() / "device" / "driver", ec).filename();

		if (!ec && (driver == "amdgpu" || driver == "radeon" || driver == "i915" || driver == "xe")) {
			const std::string dev = "/dev/dri/" + node;
			setenv("GST_VAAPI_DRM_DEVICE", dev.c_str(), 1);
			std::cout << "[camera_stream] VA-API device " << dev << " (" << driver << ")" << std::endl;
			return;
		}
	}
}

}  // namespace

/// One camera: appsrc -> encoder -> RTP -> UDP
class Stream
{
public:
	Stream(std::string topic, std::string host, std::vector<int> ports, int bitrate1080p, int fps)
		: _topic(std::move(topic)), _host(std::move(host)), _ports(std::move(ports)), _bitrate1080p(bitrate1080p), _fps(fps) {}

	~Stream() { stop(); }

	const std::string &topic() const { return _topic; }

	/// Build and start the pipeline with the given encoder, false on failure.
	bool start(const Encoder &enc, int width, int height)
	{
		stop();
		const int bitrate = std::max(1000, static_cast<int>(_bitrate1080p * double(width) * height / (1920.0 * 1080.0)));
		std::string clients;

		for (int port : _ports) {
			clients += (clients.empty() ? "" : ",") + _host + ":" + std::to_string(port);
		}

		std::string encoder = replaceAll(replaceAll(enc.description, "%b", std::to_string(bitrate)),
						 "%g", std::to_string(_fps));
		std::ostringstream desc;
		desc << "appsrc name=src is-live=true format=time do-timestamp=true"
		     << " caps=video/x-raw,format=RGB,width=" << width << ",height=" << height << ",framerate=" << _fps << "/1"
		     << " ! queue leaky=downstream max-size-buffers=2 ! videoconvert ! " << encoder
		     << " ! h264parse name=parse config-interval=-1 ! rtph264pay config-interval=1 pt=96 mtu=1400"
		     << " ! multiudpsink clients=" << clients << " sync=false async=false";

		GError *err = nullptr;
		GstElement *pipeline = gst_parse_launch(desc.str().c_str(), &err);

		if (!pipeline || err) {
			gzerr << "[camera_stream] " << enc.name << ": " << (err ? err->message : "parse failed") << std::endl;

			if (err) {
				g_error_free(err);
			}

			if (pipeline) {
				gst_object_unref(pipeline);
			}

			return false;
		}

		if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
			gzerr << "[camera_stream] " << enc.name << ": cannot start the pipeline" << std::endl;
			gst_object_unref(pipeline);
			return false;
		}

		// count encoded frames (one buffer per frame after h264parse)
		GstElement *parse = gst_bin_get_by_name(GST_BIN(pipeline), "parse");
		GstPad *pad = gst_element_get_static_pad(parse, "src");
		gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, [](GstPad *, GstPadProbeInfo *, gpointer self) {
			static_cast<Stream *>(self)->_framesOut++;
			return GST_PAD_PROBE_OK;
		}, this, nullptr);
		gst_object_unref(pad);
		gst_object_unref(parse);

		std::lock_guard<std::mutex> lock(_mutex);
		_pipeline = pipeline;
		_src = gst_bin_get_by_name(GST_BIN(pipeline), "src");
		_width = width;
		_height = height;
		_encoder = enc.name;
		std::cout << "[camera_stream] " << _topic << " -> udp://" << clients << " (" << width << "x" << height
			  << ", " << enc.name << ", " << bitrate << " kbps)" << std::endl;
		return true;
	}

	void stop()
	{
		std::lock_guard<std::mutex> lock(_mutex);

		if (_src) {
			gst_object_unref(_src);
			_src = nullptr;
		}

		if (_pipeline) {
			gst_element_set_state(_pipeline, GST_STATE_NULL);
			gst_object_unref(_pipeline);
			_pipeline = nullptr;
		}
	}

	void push(const gz::msgs::Image &img)
	{
		const auto t0 = std::chrono::steady_clock::now();

		if (_lastIn.time_since_epoch().count()) {
			const double dt = std::chrono::duration<double>(t0 - _lastIn).count();
			_minInterval = std::min(_minInterval, dt);
			_maxInterval = std::max(_maxInterval, dt);
		}

		_lastIn = t0;
		_framesIn++;
		std::lock_guard<std::mutex> lock(_mutex);

		if (!_src || static_cast<int>(img.width()) != _width || static_cast<int>(img.height()) != _height
		    || img.pixel_format_type() != gz::msgs::PixelFormatType::RGB_INT8) {
			return;
		}

		const std::string &data = img.data();
		GstBuffer *buf = gst_buffer_new_allocate(nullptr, data.size(), nullptr);
		gst_buffer_fill(buf, 0, data.data(), data.size());
		gst_app_src_push_buffer(GST_APP_SRC(_src), buf);  // takes ownership
		_pushSeconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
	}

	/// Statistics since the last call, one line.
	std::string stats(double seconds)
	{
		std::ostringstream o;
		const long in = _framesIn.exchange(0), out = _framesOut.exchange(0);
		o.setf(std::ios::fixed);
		o.precision(1);
		o << _topic << ": in " << in / seconds << " fps (interval " << (in > 1 ? _minInterval * 1000 : 0) << ".."
		  << (in > 1 ? _maxInterval * 1000 : 0) << " ms), out " << out / seconds << " fps, hand-over "
		  << (in ? _pushSeconds / in * 1000 : 0) << " ms/frame";
		_minInterval = 1e9;
		_maxInterval = 0;
		_pushSeconds = 0;
		return o.str();
	}

	/// Error message if the pipeline failed since the last call, empty otherwise.
	std::string pollError()
	{
		std::lock_guard<std::mutex> lock(_mutex);

		if (!_pipeline) {
			return {};
		}

		GstBus *bus = gst_element_get_bus(_pipeline);
		GstMessage *msg = gst_bus_pop_filtered(bus, GstMessageType(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
		gst_object_unref(bus);

		if (!msg) {
			return {};
		}

		std::string text = "end of stream";

		if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
			GError *err = nullptr;
			gst_message_parse_error(msg, &err, nullptr);
			text = err ? err->message : "unknown error";

			if (err) {
				g_error_free(err);
			}
		}

		gst_message_unref(msg);
		return text;
	}

	const std::string &encoder() const { return _encoder; }

private:
	std::string _topic, _host;
	std::vector<int> _ports;
	int _bitrate1080p, _fps;
	int _width{0}, _height{0};
	std::string _encoder;
	std::mutex _mutex;
	GstElement *_pipeline{nullptr};
	GstElement *_src{nullptr};
	std::atomic<long> _framesIn{0}, _framesOut{0};
	std::chrono::steady_clock::time_point _lastIn{};
	double _minInterval{1e9}, _maxInterval{0}, _pushSeconds{0};
};

class CameraStream : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate
{
public:
	~CameraStream() override
	{
		_running = false;

		if (_worker.joinable()) {
			_worker.join();
		}

		std::lock_guard<std::mutex> lock(_mutex);
		_streams.clear();
	}

	void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
	{
		if (auto name = ecm.Component<gz::sim::components::Name>(entity)) {
			_world = name->Data();
		}

		auto get = [&](const char *key, const std::string &fallback) {
			return sdf->HasElement(key) ? sdf->Get<std::string>(key) : fallback;
		};

		_host = getenvOr("CAMERA_STREAM_HOST", get("udp_host", "127.0.0.1"));
		_basePort = std::stoi(getenvOr("CAMERA_STREAM_PORT", get("udp_port", "5600")));
		_bitrate = std::stoi(get("bitrate_kbps", "8000"));

		// fixed ports: name=port[+port],...
		std::stringstream ps(get("ports", ""));

		for (std::string item; std::getline(ps, item, ',');) {
			const auto eq = item.find('=');

			if (eq == std::string::npos) {
				continue;
			}

			std::vector<int> ports;
			std::stringstream pp(item.substr(eq + 1));

			for (std::string port; std::getline(pp, port, '+');) {
				ports.push_back(std::stoi(port));
				_usedPorts.insert(ports.back());
			}

			_fixedPorts.emplace_back(item.substr(0, eq), ports);
		}
		_fps = std::stoi(get("fps", "30"));
		_probeTimeout = std::stoi(get("probe_timeout", "6"));
		_filter = std::regex(get("camera_filter", ".*"));

		std::string list = getenvOr("CAMERA_STREAM_ENCODERS", get("encoders", "auto"));

		if (list == "auto") {
			list = "nvenc,nvcuda,vaapi,va,qsv,x264";
		}

		std::stringstream ss(list);

		for (std::string item; std::getline(ss, item, ',');) {
			_wanted.push_back(item);
		}

		gst_init(nullptr, nullptr);
		selectVaapiDevice();
		_running = true;
		_worker = std::thread(&CameraStream::run, this);
	}

	void PostUpdate(const gz::sim::UpdateInfo &, const gz::sim::EntityComponentManager &) override {}

private:
	/// Discover cameras, pick the encoder, watch the pipelines. Runs next to the simulation.
	void run()
	{
		const std::regex imageTopic("^/world/([^/]+)/model/.+/sensor/[^/]+/image$");
		auto lastScan = std::chrono::steady_clock::now() - std::chrono::seconds(10);
		const bool printStats = getenvOr("CAMERA_STREAM_STATS", "0") == "1";
		auto lastStats = std::chrono::steady_clock::now();

		while (_running) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));

			if (std::chrono::steady_clock::now() - lastScan > std::chrono::seconds(2)) {
				lastScan = std::chrono::steady_clock::now();
				std::vector<std::string> topics;
				_node.TopicList(topics);

				for (const auto &t : topics) {
					std::smatch m;

					if (std::regex_match(t, m, imageTopic) && m[1] == _world && std::regex_search(t, _filter)
					    && !_known.count(t)) {
						_known[t] = true;
						addCamera(t);
					}
				}
			}

			std::lock_guard<std::mutex> lock(_mutex);
			const double sinceStats = std::chrono::duration<double>(std::chrono::steady_clock::now() - lastStats).count();

			if (printStats && sinceStats >= 5.0) {
				for (auto &[topic, s] : _streams) {
					std::cout << "[camera_stream] " << s.stream->stats(sinceStats) << std::endl;
				}

				lastStats = std::chrono::steady_clock::now();
			}

			for (auto &[topic, s] : _streams) {
				const std::string error = s.stream->pollError();

				if (!error.empty()) {
					gzerr << "[camera_stream] " << topic << " (" << s.stream->encoder() << ") failed: " << error << std::endl;
					restart(s);
				}
			}
		}
	}

	struct Entry {
		std::unique_ptr<Stream> stream;
		int width{0}, height{0};
		size_t encoder{0};  // index in _working
	};

	void addCamera(const std::string &topic)
	{
		{
			std::lock_guard<std::mutex> lock(_mutex);
			std::vector<int> ports;

			for (const auto &[name, fixed] : _fixedPorts) {
				if (topic.find("/" + name + "/") != std::string::npos) {
					ports = fixed;
					break;
				}
			}

			if (ports.empty()) {  // next free port
				int port = _basePort;

				while (_usedPorts.count(port)) {
					port++;
				}

				_usedPorts.insert(port);
				ports.push_back(port);
			}

			Entry &e = _streams[topic];
			e.stream = std::make_unique<Stream>(topic, _host, ports, _bitrate, _fps);
		}

		_node.Subscribe(topic, &CameraStream::onImage, this);
	}

	void onImage(const gz::msgs::Image &img, const gz::transport::MessageInfo &info)
	{
		std::unique_lock<std::mutex> lock(_mutex);
		auto it = _streams.find(info.Topic());

		if (it == _streams.end()) {
			return;
		}

		Entry &e = it->second;

		if (e.width == 0) {  // first frame: size known, choose the encoder
			e.width = static_cast<int>(img.width());
			e.height = static_cast<int>(img.height());
			lock.unlock();
			probeEncoders(e.width, e.height);
			lock.lock();
			restart(e, true);
		}

		Stream *stream = e.stream.get();
		lock.unlock();
		stream->push(img);
	}

	/// Test-run the wanted encoders once (separate process + timeout), keep the working ones in order.
	void probeEncoders(int width, int height)
	{
		std::lock_guard<std::mutex> probeLock(_probeMutex);

		if (_probed) {
			return;
		}

		for (const auto &name : _wanted) {
			const Encoder *enc = nullptr;

			for (const auto &e : allEncoders()) {
				if (e.name == name) {
					enc = &e;
				}
			}

			if (!enc) {
				gzwarn << "[camera_stream] unknown encoder " << name << std::endl;
				continue;
			}

			bool available = true;

			for (const auto &factory : enc->elements) {
				GstElementFactory *f = gst_element_factory_find(factory.c_str());
				available = available && f;

				if (f) {
					gst_object_unref(f);
				}
			}

			if (!available) {
				continue;
			}

			const std::string pipeline = "videotestsrc num-buffers=15 ! video/x-raw,format=RGB,width=" + std::to_string(width)
						     + ",height=" + std::to_string(height) + ",framerate=" + std::to_string(_fps)
						     + "/1 ! videoconvert ! "
						     + replaceAll(replaceAll(enc->description, "%b", std::to_string(_bitrate)), "%g",
								  std::to_string(_fps))
						     + " ! h264parse ! fakesink";
			const std::string cmd = "timeout -s KILL " + std::to_string(_probeTimeout) + " gst-launch-1.0 -q " + pipeline
						+ " > /dev/null 2>&1";
			const bool ok = std::system(cmd.c_str()) == 0;
			std::cout << "[camera_stream] encoder " << enc->name << ": " << (ok ? "works" : "does not work here") << std::endl;

			if (ok) {
				_working.push_back(*enc);
			}
		}

		if (_working.empty()) {
			gzerr << "[camera_stream] no working H.264 encoder, camera streams disabled" << std::endl;
		}

		_probed = true;
	}

	/// (Re)start a stream: with the current encoder the first time, else with the next working one.
	void restart(Entry &e, bool first = false)
	{
		if (!first) {
			e.encoder++;
		}

		while (e.encoder < _working.size()) {
			if (e.stream->start(_working[e.encoder], e.width, e.height)) {
				return;
			}

			e.encoder++;
		}

		e.stream->stop();
		gzerr << "[camera_stream] " << e.stream->topic() << ": no encoder left" << std::endl;
	}

	gz::transport::Node _node;
	std::string _world, _host;
	int _basePort{5600}, _bitrate{8000}, _fps{30}, _probeTimeout{6};
	std::regex _filter;
	std::vector<std::string> _wanted;
	std::vector<std::pair<std::string, std::vector<int>>> _fixedPorts;
	std::set<int> _usedPorts;
	std::vector<Encoder> _working;
	bool _probed{false};
	std::mutex _probeMutex;
	std::mutex _mutex;
	std::map<std::string, Entry> _streams;
	std::map<std::string, bool> _known;
	std::atomic<bool> _running{false};
	std::thread _worker;
};

}  // namespace interceptor_sim

GZ_ADD_PLUGIN(interceptor_sim::CameraStream, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(interceptor_sim::CameraStream, "interceptor_sim::CameraStream")
