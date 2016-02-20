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


#define FAST_THRESHOLD 40
#define MASK_KEEPOUT_SIZE 5

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

	// KLT algorithm
	cv::Mat 				_image_prev;
	cv::Mat					_image_curr;
	cv::TermCriteria 			_term_crit;
	std::vector<cv::Point2f> 		_features_prev;
	std::vector<cv::Point2f> 		_features_curr;
	std::vector<uint8_t> 			_features_tracked;
	std::vector<float> 			_error;
	std::unique_ptr<std::vector<cv::Point2f>> _new_points;

	float 					_angular_flow_x;
	float					_angular_flow_y;

	cv::VideoCapture _imagesrc;

	void		task_main();

	/**
	 * Publish optical flow estimates
	 */
	void		publish_flow();


	std::unique_ptr<std::vector<cv::Point2f>> extract_features(const cv::Mat &inputImage, const cv::Mat &mask,
					       int numPoints);

	std::unique_ptr<std::vector<cv::Point2f>> optimize_points(const cv::Mat &image, std::vector<cv::Point2f> &currentPoints,
					       int numPoints);

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
	_image_prev(),
	_image_curr(),
	_term_crit(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
	_features_prev(),
	_features_curr(),
	_features_tracked(),
	_error(),
	_new_points(),
	_angular_flow_x(0.0f),
	_angular_flow_y(0.0f),
	_imagesrc(0)
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

	unsigned int _frame_seq = 0;

	// Make these params TODO
	int numPoints = 50;
	float threshold = 1.0;
	float focal_length = 16.0;

	// Get initial image frames
	_imagesrc >> _image_prev;
	_imagesrc >> _image_curr;

	// Get initial features
	std::unique_ptr<std::vector<cv::Point2f>> initialPoints = extract_features(_image_prev, cv::Mat() , numPoints);

	for (auto pt : *initialPoints) {
		_features_prev.push_back(pt);
	}

	while (!_task_should_exit && _image_curr.data != NULL) {

		//full_loop_timer.start();
		//tracker_timer.start();

		cv::calcOpticalFlowPyrLK(_image_prev, _image_curr, _features_prev, _features_curr, _features_tracked, _error,
					 cv::Size(31,
						  31), 3, _term_crit,
					 cv::OPTFLOW_LK_GET_MIN_EIGENVALS, threshold);

		//tracker_timer.stop();

		//if(pm_on) {
		//	point_management_timer.start();
		//}

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
				// delete untracked points
				_features_curr.erase(_features_curr.begin() + pt);
			}
		}

		_angular_flow_x = atan2(pixel_flow_x_integral / counter, focal_length);
		_angular_flow_y = atan2(pixel_flow_y_integral / counter, focal_length);

		if (_features_curr.size() == 0) { break; }

		_features_prev = _features_curr;
		_image_prev = _image_curr;

		// Point optimizer
		_new_points = optimize_points(_image_curr, _features_curr, numPoints);

		int num_points_added = 0;

		if (_new_points != NULL) {
			for (auto pt : *_new_points) {
				_features_prev.push_back(pt);
			}

			num_points_added = _new_points->size();
			_new_points->clear();
		}

		warnx("added %i pts", num_points_added);

		//point_management_timer.stop();

		//full_loop_timer.stop();

		// Display image with points
		//imageToBGR(_image_curr);

		for (auto pt : _features_curr) {
			cv::circle(_image_curr, pt, 1, cv::Scalar(0, 255, 0), 2, 8, 0);
		}

		cv::imshow("Flow tracking", _image_curr);
		cv::waitKey(10);

		// TODO overlay algorithm performance on image

		// next image frame
		_imagesrc >> _image_curr;

		_frame_seq++;

		//const unsigned sleeptime_us = 9500;

		// TODO sleep here?

		//hrt_abstime last_run = hrt_absolute_time();
		//float dt_runs = sleeptime_us / 1e6f;

	}

	warnx("exiting.");

	_main_task = -1;
	_exit(0);
}


std::unique_ptr<std::vector<cv::Point2f>>
				       OpticalFlow::extract_features(const cv::Mat &inputImage, const cv::Mat &mask, int numPoints)
{
	//if (numPoints < 0) { numPoints = maxPoints; }

	std::vector<cv::KeyPoint> kPoints;
	std::unique_ptr<std::vector<cv::Point2f>> points(new std::vector<cv::Point2f>);

	cv::FAST(inputImage, kPoints, FAST_THRESHOLD, true);

	// Sort the keypoints by their "strength" (response)
	std::sort(kPoints.begin(), kPoints.end(), [](const cv::KeyPoint & kp1, const cv::KeyPoint & kp2) { return kp1.response > kp2.response; });

	// Copy the best points to the vector of new points, checking them against the mask.
	unsigned int index = 0;

	if (!mask.empty()) {
		while (points->size() < (unsigned int)numPoints && (unsigned int)index < kPoints.size()) {
			if (mask.at<float>(kPoints[index].pt.x, kPoints[index].pt.y) > 0) { // check TODO was !=
				points->push_back(kPoints[index].pt);
			}

			index++;
		}

	} else {
		while (points->size() < (unsigned int)numPoints) {
			points->push_back(kPoints[index].pt);
			index++;
		}
	}

	return points;
}

std::unique_ptr<std::vector<cv::Point2f>>
				       OpticalFlow::optimize_points(const cv::Mat &image, std::vector<cv::Point2f> &currentPoints, int numPoints)
{
	std::unique_ptr<std::vector<cv::Point2f>> _new_points;

	if (currentPoints.size() < (unsigned int)numPoints) {
		// Construct mask
		cv::Point2f halfDiagonal(MASK_KEEPOUT_SIZE, MASK_KEEPOUT_SIZE);
		cv::Mat mask(image.size(), image.type(), cv::Scalar(255));

		for (auto pt : currentPoints) {
			cv::rectangle(mask, (pt - halfDiagonal), (pt + halfDiagonal), cv::Scalar(0), -1, 8, 0);
		}

		// Get vector of new points from extractor
		int newFeaturesRequired = numPoints - currentPoints.size();
		_new_points = extract_features(image, mask, newFeaturesRequired);
	}

	return _new_points;
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
