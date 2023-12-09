//
// Created by gaoxiang on 19-5-4.
//

#include "myslam/visual_odometry.h"
#include <gflags/gflags.h>

// DEFINE_string是一个宏，用于定义一个gflags的变量，其参数依次为变量名、默认值，
//你可以使用 --config_file 参数来指定配置文件的路径。
//如果你没有指定--config_file，那么它的值就会是默认值
DEFINE_string(config_file, "/home/cw/Slam/ch13/config/default.yaml",
              "config file path");

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry(FLAGS_config_file));
  assert(vo->Init() == true);
  vo->Run();

  return 0;
}
