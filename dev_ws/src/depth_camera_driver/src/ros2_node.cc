#include <sys/sem.h>
#include <sys/shm.h>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "ros2_device.h"
#include "glog/logging.h"
#include "aurora900_ros2_device.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_colorlogtostderr = true;
  FLAGS_logbuflevel = -1;  // 不缓存
  google::SetStderrLogging(google::GLOG_INFO);

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("aurora");
  LOG(WARNING) << "hello aurora 900/930";
  std::shared_ptr<RosDevice> device(new Aurora900RosDevice(node));

  DeptrumRosDeviceParams param = {};
  param = device->params_;
  int boot_order = param.boot_order;
  if (param.device_numbers > 1) {
    int shmid;
    char* shm = NULL;
    if (boot_order == 1) {
      if ((shmid = shmget((key_t) STELLAR_PID, 1, 0666 | IPC_CREAT)) == -1)
        LOG(ERROR) << "BOOT_ORDER_" << param.boot_order
                   << "::Create Share Memory Error:" << strerror(errno);
      shm = (char*) shmat(shmid, 0, 0);
      *shm = 1;
      device->InitDevice();
      LOG(INFO) << "BOOT_ORDER_" << param.boot_order << "::Device " << param.serial_number.c_str()
                << "already open";
      *shm = 2;
    } else {
      if ((shmid = shmget((key_t) STELLAR_PID, 1, 0666 | IPC_CREAT)) == -1)
        LOG(ERROR) << "BOOT_ORDER_" << param.boot_order
                   << "::Create Share Memory Error:" << strerror(errno);
      shm = (char*) shmat(shmid, 0, 0);

      while (*shm != boot_order)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

      device->InitDevice();
      LOG(INFO) << "BOOT_ORDER_" << param.boot_order << "::Device" << param.serial_number.c_str()
                << "already open.";
      *shm = boot_order + 1;
    }
    if (boot_order == param.device_numbers) {
      if (shmdt(shm) == -1)
        LOG(ERROR) << "BOOT_ORDER_" << param.boot_order << " shmdt failed:" << strerror(errno);
      if (shmctl(shmid, IPC_RMID, 0) == -1)
        LOG(ERROR) << "BOOT_ORDER_" << param.boot_order
                   << "shmctl(IPC_RMID) failed :" << strerror(errno);
    } else {
      if (shmdt(shm) == -1)
        LOG(ERROR) << "BOOT_ORDER_" << param.boot_order << "shmdt failed:" << strerror(errno);
    }
  } else {
    device->InitDevice();
  }

  device->Start();
  rclcpp::spin(node);
  rclcpp::shutdown();
  LOG(INFO) << "process quit succeed";
  return 0;
}
