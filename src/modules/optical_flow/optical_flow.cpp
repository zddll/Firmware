/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file optical_flow.cpp
 *
 * KLT Optical Flow Tracker
 *
 * @author Mohammed Kabir <mhkabir98@gmail.com>
 */

#include <px4_config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <sys/ioctl.h>
#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/optical_flow.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <dataman/dataman.h>
#include <mathlib/mathlib.h>
#include <mavlink/mavlink_log.h>

#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <vector>

/**
 * optical_flow app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int optical_flow_main(int argc, char *argv[]);

class OpticalFlow
{
public:
	/**
	 * Constructor
	 */
	OpticalFlow();

	/**
	 * Destructor, also kills task.
	 */
	~OpticalFlow();

	/**
	 * Start the task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	/**
	 * Display status.
	 */
	void		status();

private:
	bool		_task_should_exit;		/**< if true, task should exit */
	int		_main_task;			/**< handle for task */
	int		_mavlink_fd;


	unsigned int 				_frame_seq;
	uint64_t 				_frame_stamp;
	uint64_t 				_integration_time;

	bool					_visualise_tracking;

	// Optical flow algorithm
	int					_max_features;
	int					_mask_window_size;
	float					_FAST_theshold;
	cv::Mat 				_image_prev;
	cv::Mat					_image_curr;
	std::vector<cv::Point2f> 		_features_prev;
	std::vector<cv::Point2f> 		_features_curr;
	std::vector<uint8_t> 			_features_tracked;
	std::vector<float> 			_error;
	std::unique_ptr<std::vector<cv::Point2f>> _initial_points;
	std::unique_ptr<std::vector<cv::Point2f>> _new_points;

	float 					_focal_length;
	float 					_angular_flow_x;
	float					_angular_flow_y;

	cv::VideoCapture 			_imagesrc;

	orb_advert_t				_flow_pub;

	void		task_main();

	/**
	 * Publish optical flow estimates
	 */
	void		publish_flow();

	void		init_tracker();

	void		run_tracker();

	std::unique_ptr<std::vector<cv::Point2f>> extract_features(const cv::Mat &inputImage, const cv::Mat &mask,
					       int num_features);

	std::unique_ptr<std::vector<cv::Point2f>> optimize_points(const cv::Mat &image, std::vector<cv::Point2f> &currentPoints,
					       int num_features);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);
};

namespace optical_flow
{
OpticalFlow	*g_optical_flow;
}

OpticalFlow::OpticalFlow() :

	_task_should_exit(false),
	_main_task(-1),
	_mavlink_fd(-1),
	_frame_seq(0),
	_frame_stamp(0),
	_integration_time(0),
	_visualise_tracking(true),
	_max_features(200),
	_mask_window_size(5),
	_FAST_theshold(40),
	_image_prev(cv::Mat(640, 480, CV_8UC3)),
	_image_curr(cv::Mat(640, 480, CV_8UC3)),
	_features_prev(),
	_features_curr(),
	_features_tracked(),
	_error(),
	_initial_points(),
	_new_points(),
	_focal_length(1.0f),
	_angular_flow_x(0.0f),
	_angular_flow_y(0.0f),
	_imagesrc(0),
	_flow_pub(nullptr)
{
}

OpticalFlow::~OpticalFlow()
{
	if (_main_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_main_task);
				break;
			}
		} while (_main_task != -1);
	}

	optical_flow::g_optical_flow = nullptr;
}

int
OpticalFlow::start()
{
	ASSERT(_main_task == -1);

	/* start the task */
	_main_task = px4_task_spawn_cmd("optical_flow",
					SCHED_DEFAULT,
					SCHED_PRIORITY_DEFAULT + 15,
					1500,
					(px4_main_t)&OpticalFlow::task_main_trampoline,
					nullptr);

	if (_main_task < 0) {
		warnx("task start failed");
		return -errno;
	}

	return OK;
}


void
OpticalFlow::status()
{

}

void
OpticalFlow::publish_flow()
{

	struct optical_flow_s flow = {};

	flow.timestamp = _frame_stamp;
	flow.pixel_flow_x_integral = _angular_flow_x;
	flow.pixel_flow_y_integral = _angular_flow_y;
	flow.integration_timespan = _integration_time;
	flow.quality = _features_curr.size() / _max_features * 255;

	int flow_inst = 1;
	orb_publish_auto(ORB_ID(optical_flow), &_flow_pub, &flow, &flow_inst, ORB_PRIO_DEFAULT);

}

void
OpticalFlow::task_main()
{

	_mavlink_fd = px4_open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(_mavlink_fd, "[optical_flow] started");

	//bool updated = false;

	//struct parameter_update_s update;
	//memset(&update, 0, sizeof(update));
	//int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));

	init_tracker();

	while (!_task_should_exit && _image_curr.data != NULL) {
		run_tracker();
	}

	warnx("exiting.");

	_main_task = -1;
	_exit(0);
}

void
OpticalFlow::init_tracker()
{
	// Get initial image frames
	_frame_stamp = hrt_absolute_time();
	_imagesrc >> _image_prev;
	_imagesrc >> _image_curr;

	// Convert to greyscale
	cvtColor(_image_prev, _image_prev, CV_RGB2GRAY);
	cvtColor(_image_curr, _image_curr, CV_RGB2GRAY);

	// Get initial features
	_initial_points = extract_features(_image_prev, cv::Mat() , _max_features);

	if (_initial_points != NULL) {
		for (auto pt : *_initial_points) {
			_features_prev.push_back(pt);
		}
	} 
}

void
OpticalFlow::run_tracker()
{
	// Run KLT algorithm
	cv::calcOpticalFlowPyrLK(_image_prev, _image_curr, _features_prev, _features_curr, _features_tracked, _error);

	// Compute optical flow
	float pixel_flow_x_integral = 0.0f;
	float pixel_flow_y_integral = 0.0f;

	int counter = 0;

	for (unsigned int pt = 0; pt < _features_tracked.size(); pt++) {
		if (_features_tracked[pt] != 0) {
			for (int i = 0; i < _features_curr.size(); i++) {
				pixel_flow_x_integral += _features_curr[i].x - _features_prev[i].x;
				pixel_flow_y_integral += _features_curr[i].y - _features_prev[i].y;
				counter++;
			}

		} else {
			// Delete untracked points
			_features_curr.erase(_features_curr.begin() + pt);
		}
	}

	if (_features_curr.empty()) {
		// Lost tracking, start over
		init_tracker();
		return;
	}

	// Calculate flow rates
	_angular_flow_x = atan2(pixel_flow_x_integral / counter, _focal_length);
	_angular_flow_y = atan2(pixel_flow_y_integral / counter, _focal_length);

	_integration_time = hrt_absolute_time() - _frame_stamp;

	// Publish estimates
	publish_flow();

	_features_prev = _features_curr;
	_image_prev = _image_curr;

	// Point optimizer
	_new_points = optimize_points(_image_curr, _features_curr, _max_features);

	if (_new_points != NULL) {
		for (auto pt : *_new_points) {
			_features_prev.push_back(pt);
		}

		_new_points->clear();
	}
	
	// Display current image with points
	if (_visualise_tracking) {
		cvtColor(_image_curr, _image_curr, CV_GRAY2BGR);

		for (auto pt : _features_curr) {
			cv::circle(_image_curr, pt, 1, cv::Scalar(0, 255, 0), 2, 8, 0);
		}

		// TODO overlay algorithm performance on image
		cv::imshow("Flow tracking", _image_curr);
		cv::waitKey(10);
	}

	// next image frame
	_frame_stamp = hrt_absolute_time();

	_imagesrc >> _image_curr;
	cvtColor(_image_curr, _image_curr, CV_RGB2GRAY);

	_frame_seq++;

}

std::unique_ptr<std::vector<cv::Point2f>>
				       OpticalFlow::extract_features(const cv::Mat &input_image, const cv::Mat &mask, int num_features)
{

	std::vector<cv::KeyPoint> keypoints;
	std::unique_ptr<std::vector<cv::Point2f>> points(new std::vector<cv::Point2f>);

	cv::FAST(input_image, keypoints, _FAST_theshold, true);

	// Sort the keypoints by their "strength" (response)
	std::sort(keypoints.begin(), keypoints.end(), [](const cv::KeyPoint & kp1, const cv::KeyPoint & kp2) { return kp1.response > kp2.response; });

	// Copy the best points to the vector of new points, checking them against the mask.
	unsigned int index = 0;

	if (!mask.empty() && !keypoints.empty()) {
		while (points->size() < (unsigned int)num_features && (unsigned int)index < keypoints.size()) {
			if (mask.at<float>(keypoints[index].pt.x, keypoints[index].pt.y) > 0) {
				points->push_back(keypoints[index].pt);
			}

			index++;
		}

	} else if (!keypoints.empty()) {
		while (points->size() < (unsigned int)num_features) {
			points->push_back(keypoints[index].pt);
			index++;
		}
	}

	return points;
}

std::unique_ptr<std::vector<cv::Point2f>>
				       OpticalFlow::optimize_points(const cv::Mat &image, std::vector<cv::Point2f> &current_points, int num_features)
{
	std::unique_ptr<std::vector<cv::Point2f>> new_points;

	if (current_points.size() < (unsigned int)num_features) {
		// Construct mask
		cv::Point2f half_diagonal(_mask_window_size, _mask_window_size);
		cv::Mat mask(image.size(), image.type(), cv::Scalar(255));

		for (auto pt : current_points) {
			cv::rectangle(mask, (pt - half_diagonal), (pt + half_diagonal), cv::Scalar(0), -1, 8, 0);
		}

		// Get vector of new points from extractor
		int new_fts_required = num_features - current_points.size();
		new_points = extract_features(image, mask, new_fts_required);
	}

	return new_points;
}


void
OpticalFlow::task_main_trampoline(int argc, char *argv[])
{
	optical_flow::g_optical_flow->task_main();
}

static void usage()
{
	warnx("usage: optical_flow {start|stop|status}");
}

int optical_flow_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (optical_flow::g_optical_flow != nullptr) {
			warnx("already running");
		}

		optical_flow::g_optical_flow = new OpticalFlow;

		if (optical_flow::g_optical_flow == nullptr) {
			warnx("alloc failed");
		}

		if (OK != optical_flow::g_optical_flow->start()) {
			delete optical_flow::g_optical_flow;
			optical_flow::g_optical_flow = nullptr;
			warnx("start failed");
			return 0;
		}

		return 0;
	}

	if (optical_flow::g_optical_flow == nullptr) {
		warnx("not running");
	}

	if (!strcmp(argv[1], "stop")) {
		delete optical_flow::g_optical_flow;
		optical_flow::g_optical_flow = nullptr;

	} else if (!strcmp(argv[1], "status")) {
		optical_flow::g_optical_flow->status();

	} else {
		usage();
	}

	return 0;
}
