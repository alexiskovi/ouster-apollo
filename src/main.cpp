#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <limits>
#include <memory>
#include <string>

#include "modules/drivers/ouster/include/ouster/os1.h"
#include "modules/drivers/ouster/include/ouster/os1_packet.h"
#include "modules/drivers/ouster/include/ouster/os1_util.h"
#include "modules/drivers/ouster/include/ouster/point_os1.h"

#include "modules/transform/buffer.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "cyber/base/concurrent_object_pool.h"
#include "cyber/base/object_pool.h"

#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "modules/common/util/message_util.h"

using apollo::cyber::Rate;
using apollo::cyber::Time;

namespace OS1 = ouster::OS1;

using PointOS1 = ouster::OS1::PointOS1;
using CloudOS1 = pcl::PointCloud<PointOS1>;
using namespace apollo::cyber;
using namespace apollo::drivers;
using namespace apollo::cyber::base;
using Points = Eigen::Array<double, Eigen::Dynamic, 3, Eigen::RowMajor>;

std::vector<double> beam_altitude_angles;
std::vector<double> beam_azimuth_angles;

uint64_t n_lidar_packets = 0;
uint64_t n_imu_packets = 0;

uint64_t lidar_col_0_ts = 0;
uint64_t imu_ts = 0;

float lidar_col_0_h_angle = 0.0;
float imu_av_z = 0.0;
float imu_la_y = 0.0;
std::shared_ptr<Writer<PointCloud>> writer_;
std::shared_ptr<CCObjectPool<PointCloud>> point_cloud_pool_ = nullptr;
int pool_size_ = 8;

bool Init() {
    std::shared_ptr<apollo::cyber::Node> node_(apollo::cyber::CreateNode("ouster_talker"));
    writer_ = node_->CreateWriter<PointCloud>("/apollo/sensor/lidar128/compensator/PointCloud2");
    point_cloud_pool_.reset(new CCObjectPool<PointCloud>(pool_size_));
    point_cloud_pool_->ConstructAll();
    for (int i = 0; i < pool_size_; i++) {
        auto point_cloud = point_cloud_pool_->GetObject();
        if (point_cloud == nullptr) {
            AERROR << "fail to get object, i: " << i;
            return false;
        }
        point_cloud->mutable_point()->Reserve(140000);
    }
    AINFO << "Point cloud comp convert init success";
    return true;
}

bool Proc(CloudOS1 &cloud, double m_time, uint64_t seq) {
    std::shared_ptr<PointCloud> point_cloud_out = point_cloud_pool_->GetObject();
    if (point_cloud_out == nullptr) {
        AWARN << "point cloud pool return nullptr, will be create new.";
        point_cloud_out = std::make_shared<PointCloud>();
        point_cloud_out->mutable_point()->Reserve(140000);
    }
    if (point_cloud_out == nullptr) {
        AWARN << "point cloud out is nullptr";
        return false;
    }
    point_cloud_out->Clear();
    point_cloud_out->set_measurement_time(m_time);
    point_cloud_out->mutable_header()->set_frame_id("velodyne");
    point_cloud_out->set_height(10);
    point_cloud_out->set_frame_id("lidar128");
    point_cloud_out->set_width(1024);
    point_cloud_out->mutable_header()->set_sequence_num(seq);
    for (const auto& it: cloud) {
        if((std::abs(it.x)>0.001)||(std::abs(it.y)>0.001)||(std::abs(it.z)>0.001)){
            apollo::drivers::PointXYZIT* cyber_point = point_cloud_out->add_point();
            cyber_point->set_timestamp(Time::Now().ToSecond());
            cyber_point->set_x(it.x);
            cyber_point->set_y(it.y);
            cyber_point->set_z(it.z);
            cyber_point->set_intensity((int)(it.intensity / 1724.0 * 255.0));
        }
    }

    if (point_cloud_out == nullptr || point_cloud_out->point().empty()) {
        AWARN << "point_cloud_out convert is empty.";
        return false;
    }
    writer_->Write(point_cloud_out);
    return true;
}


int connection_loop(std::shared_ptr<OS1::client> cli, std::string filename) {
    std::fstream file;
    if (cli) {
        file.open(filename, std::ios::binary  | file.out);
	} else {
        file.open(filename, std::ios::binary  | file.in);
    }

    double m_time;
    uint64_t seq = 0;
    uint32_t H = OS1::pixels_per_column;
    uint32_t W = OS1::n_cols_of_lidar_mode(
        OS1::lidar_mode_of_string("1024x10"));
    uint8_t lidar_buf[OS1::lidar_packet_bytes + 1];
    if (cli) {
        auto metadata = OS1::get_metadata(*cli);
        auto info = OS1::parse_metadata(metadata);
        beam_azimuth_angles = info.beam_azimuth_angles;
        beam_altitude_angles = info.beam_altitude_angles;
    }
    auto xyz_lut = OS1::make_xyz_lut(W, H, beam_azimuth_angles, beam_altitude_angles);
    Init();
    CloudOS1 cloud{W, H};
    auto it = cloud.begin();
    auto batch_and_update = OS1::batch_to_iter<CloudOS1::iterator>(
        xyz_lut, W, H, {},
        &PointOS1::make, [&](uint64_t) {
            Proc(cloud, m_time, seq++);
        });

    while (apollo::cyber::OK()){
        bool got = false;
        if (cli) {
            auto state = OS1::poll_client(*cli);
            if (state == OS1::EXIT) {
                AINFO << "poll_client: caught signal, exiting";
                return EXIT_SUCCESS;
            }
            if (state & OS1::ERROR) {
                AERROR << "poll_client: returned error";
                return EXIT_FAILURE;
            }
            if (state & OS1::LIDAR_DATA) {
                got = OS1::read_lidar_packet(*cli, lidar_buf);
                if (file.is_open())  {
                    file.write(const_cast<char*>(reinterpret_cast<char*>(lidar_buf)), OS1::columns_per_buffer * OS1::column_bytes);  
                }
            }
        } else if (file.is_open()) {
            got = nullptr != file.read(reinterpret_cast<char*>(lidar_buf), OS1::columns_per_buffer * OS1::column_bytes);
            if (!got) {
                file.close();
                file.open(filename, std::ios::binary  | file.in);
                got = nullptr != file.read(reinterpret_cast<char*>(lidar_buf), OS1::columns_per_buffer * OS1::column_bytes);
            }
        }
        if (got) {
            m_time = Time::Now().ToSecond();
            batch_and_update(lidar_buf, it);
        }
    }
    return 0;
}

int main(int argc, char** argv) {
    std::cout << "to save from lidar to buffer: "<< argv[0] << " lidar.bin save" <<std::endl;
    std::cout << "to load from buffer instead of lidar: "<< argv[0] << " lidar.bin" <<std::endl;

    apollo::cyber::Init(argv[0]);
    if (argc==1 || argc==3) {
	    auto cli = OS1::init_client("10.5.5.77", "10.5.5.1", OS1::lidar_mode_of_string("1024x10"), 7501, 7502);
	    if (!cli) {
		    std::cerr << "Failed to connect to client."  << std::endl;
		    return -1;
	    }
	    return connection_loop(cli, argc>2? argv[1] : "" );
    }
    connection_loop(nullptr, argv[1]);

    return 0;
}
