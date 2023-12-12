#include "camtrk.h"

#include <librealsense2/rs.hpp>

//#include <concurrent_queue.h>
#include <iostream>
#include <string>
#include <map>
#include <algorithm>
#include <queue>
#include <mutex>

static rs2::pipeline __pipe;
// Create and initialize GUI related objects
static rs2::colorizer __c;                     // Helper to colorize depth images
static rs2::config __cfg;

template<typename Data>
class concurrent_queue // single consumer
{
private:
	std::queue<Data> the_queue;
	mutable std::mutex the_mutex;

	std::condition_variable the_condition_variable;
	int MAX_QUEUE;
	Data undefinedData;

public:
	concurrent_queue(int cap = 10)
	{
		MAX_QUEUE = cap;
	}

	void SetMaxQueue(int cap) {
		MAX_QUEUE = cap;
	}

	void wait_for_data()
	{
		//std::lock_guard<std::mutex> lock(the_mutex);
		std::unique_lock<std::mutex> lock(the_mutex);
		while (the_queue.empty())
		{
			the_condition_variable.wait(lock);
		}
	}

	void wait_and_pop(Data& popped_value)
	{
		//std::lock_guard<std::mutex> lock(the_mutex);
		std::unique_lock<std::mutex> lock(the_mutex);
		while (the_queue.empty())
		{
			the_condition_variable.wait(lock);
		}

		popped_value = the_queue.front();
		the_queue.pop();
	}

	//void postphone_back_and_clear(Data& popped_value)
	//{
	//	boost::mutex::scoped_lock lock(the_mutex);
	//	while (the_queue.size() < MAX_QUEUE)
	//	{
	//		the_condition_variable.wait(lock);
	//	}
	//
	//	popped_value = the_queue.back();
	//	std::queue<Data> empty_queue;
	//	std::swap(the_queue, empty_queue);
	//}

	void push(const Data& data) //enque
	{
		//boost::mutex::scoped_lock lock(the_mutex);
		//the_queue.push(data);

		std::unique_lock<std::mutex> lock(the_mutex);
		bool const was_empty = the_queue.empty();
		the_queue.push(data);
		if (the_queue.size() > MAX_QUEUE)
		{
			the_queue.pop();
		}

		lock.unlock(); // unlock the mutex

		if (was_empty)
		{
			the_condition_variable.notify_one();
		}
	}

	bool empty() const
	{
		//std::lock_guard<std::mutex> lock(the_mutex);
		std::unique_lock<std::mutex> lock(the_mutex);
		return the_queue.empty();
	}

	Data& front()
	{
		//std::lock_guard<std::mutex> lock(the_mutex);
		std::unique_lock<std::mutex> lock(the_mutex);
		if (the_queue.empty())
			return undefinedData;
		return the_queue.front();
	}

	Data const& front() const
	{
		//std::lock_guard<std::mutex> lock(the_mutex);
		std::unique_lock<std::mutex> lock(the_mutex);
		if (the_queue.empty())
			return undefinedData;
		return the_queue.front();
	}

	void pop() // deque
	{
		//std::lock_guard<std::mutex> lock(the_mutex);
		std::unique_lock<std::mutex> lock(the_mutex);
		the_queue.pop();
	}
};

namespace camtrk {
	concurrent_queue<camtrk::CamTrackInfo> __trackInfoQueue;
}
//concurrency::concurrent_queue<camtrk::CamTrackInfo> __trackInfoQueue;

// This example assumes camera with depth and color
// streams, and direction lets you define the target stream
enum class AlignDirection
{
	TO_DEPTH,
	TO_COLOR
};

AlignDirection __alignDir = AlignDirection::TO_COLOR;

// Find devices with specified streams
bool device_with_streams(std::vector <rs2_stream> stream_requests, std::string& out_serial)
{
	rs2::context ctx;
	auto devs = ctx.query_devices();
	std::vector <rs2_stream> unavailable_streams = stream_requests;
	for (auto dev : devs)
	{
		std::map<rs2_stream, bool> found_streams;
		for (auto& type : stream_requests)
		{
			found_streams[type] = false;
			for (auto& sensor : dev.query_sensors())
			{
				for (auto& profile : sensor.get_stream_profiles())
				{
					if (profile.stream_type() == type)
					{
						found_streams[type] = true;
						unavailable_streams.erase(std::remove(unavailable_streams.begin(), unavailable_streams.end(), type), unavailable_streams.end());
						if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
							out_serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
					}
				}
			}
		}
		// Check if all streams are found in current device
		bool found_all_streams = true;
		for (auto& stream : found_streams)
		{
			if (!stream.second)
			{
				found_all_streams = false;
				break;
			}
		}
		if (found_all_streams)
			return true;
	}
	// After scanning all devices, not all requested streams were found
	for (auto& type : unavailable_streams)
	{
		switch (type)
		{
		case RS2_STREAM_POSE:
		case RS2_STREAM_FISHEYE:
			std::cerr << "Connect T26X and rerun the demo" << std::endl;
			break;
		case RS2_STREAM_DEPTH:
			std::cerr << "The demo requires Realsense camera with DEPTH sensor" << std::endl;
			break;
		case RS2_STREAM_COLOR:
			std::cerr << "The demo requires Realsense camera with RGB sensor" << std::endl;
			break;
		default:
			throw std::runtime_error("The requested stream: " + std::to_string(type) + ", for the demo is not supported by connected devices!"); // stream type
		}
	}
	return false;
}

bool camtrk::InitCamTrackLib()
{
	std::string serial;
	if (!device_with_streams({ RS2_STREAM_COLOR,RS2_STREAM_DEPTH }, serial))
		return EXIT_SUCCESS;

	// Create a pipeline to easily configure and start the camera
	if (!serial.empty())
		__cfg.enable_device(serial);
	__cfg.enable_stream(RS2_STREAM_DEPTH);
	__cfg.enable_stream(RS2_STREAM_COLOR);
	__pipe.start(__cfg);

	return true;
}

void camtrk::SetAlignmentDir(bool toColorDir) 
{
	__alignDir = toColorDir ? AlignDirection::TO_COLOR : AlignDirection::TO_DEPTH;
}

bool camtrk::GetLatestCamTrackInfoSafe(CamTrackInfo& camtrkInfo)
{
	if (__trackInfoQueue.empty()) return false;
	__trackInfoQueue.wait_and_pop(camtrkInfo);
	return true;
}

bool camtrk::UpdateFrame()
{
	// Define two align objects. One will be used to align
	// to depth viewport and the other to color.
	// Creating align object is an expensive operation
	// that should not be performed in the main loop

	static rs2::align align_to_depth(RS2_STREAM_DEPTH);
	static rs2::align align_to_color(RS2_STREAM_COLOR);

	rs2::frameset frameset = __pipe.wait_for_frames();

	if (__alignDir == AlignDirection::TO_DEPTH)
	{
		// Align all frames to depth viewport
		frameset = align_to_depth.process(frameset);
	}
	else
	{
		// Align all frames to color viewport
		frameset = align_to_color.process(frameset);
	}

	// With the aligned frameset we proceed as usual
	auto depth = frameset.get_depth_frame();
	auto color = frameset.get_color_frame();
	auto colorized_depth = __c.colorize(depth);

	//Mat image(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
	CamTrackInfo camTrkInfo;
	camTrkInfo.color_w = color.as<rs2::video_frame>().get_width();
	camTrkInfo.color_h = color.as<rs2::video_frame>().get_height();
	camTrkInfo.depth_w = colorized_depth.as<rs2::video_frame>().get_width();
	camTrkInfo.depth_h = colorized_depth.as<rs2::video_frame>().get_height();

	camTrkInfo.colorBuffer.assign(camTrkInfo.color_w * camTrkInfo.color_h * 3, 0);
	memcpy(&camTrkInfo.colorBuffer[0], (void*)color.get_data(), camTrkInfo.colorBuffer.size());
	camTrkInfo.depthColorCodedBuffer.assign(camTrkInfo.depth_w * camTrkInfo.depth_h * 3, 0);
	memcpy(&camTrkInfo.depthColorCodedBuffer[0], (void*)colorized_depth.get_data(), camTrkInfo.colorBuffer.size());

	// TO DO
	// DEFAULT PROCESSING

	return true;
}

bool camtrk::DeinitCamTrackLib()
{
	return true;
}