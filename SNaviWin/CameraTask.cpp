#include "CameraTask.h"

#include "VisMtvApi.h"
#include "ApiUtility.hpp"
#include "rapidcsv/rapidcsv.h"

#include <time.h>
#include <opencv2/opencv.hpp>

namespace camtask {

	__GC* __gc = NULL;

	void InitializeTask(__GC* gcp) {
		__gc = gcp;
	}

	void CameraProcess() {
	}
}