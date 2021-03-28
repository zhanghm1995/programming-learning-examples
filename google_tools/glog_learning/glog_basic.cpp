#include <glog/logging.h>

int main(int argc, char* argv[]) {
    // Initialize Google’s logging library.
    google::InitGoogleLogging(argv[0]);

    // ...
    LOG(INFO) << "Found "  << " cookies";

    CHECK_GE(10, 3U);
    CHECK_GE(3, 5);
}