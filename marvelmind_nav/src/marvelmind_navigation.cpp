/*
Copyright (c) 2020, Marvelmind Robotics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

#include "marvelmind_nav/marvelmind_navigation.hpp"

uint32_t MarvelmindNavigation::hedge_timestamp_prev = 0;
static sem_t *sem;
void semCallback()
{
    sem_post(sem);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MarvelmindNavigation::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO_STREAM(get_logger(),std::fixed << "On activate at " << this->now().seconds());
    activateAllPublishers();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MarvelmindNavigation::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO_STREAM(get_logger(),std::fixed << "On deactivate at " << this->now().seconds());
    deactivateAllPublishers();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MarvelmindNavigation::on_error(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO_STREAM(get_logger(),std::fixed << "On error at " << this->now().seconds());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MarvelmindNavigation::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO_STREAM(get_logger(),std::fixed << "On cleanup at " << this->now().seconds());
    resetAllPublishers();
    timer_.reset();
    if (hedge != NULL)
    {
        stopMarvelmindHedge (hedge);
        destroyMarvelmindHedge (hedge);
    }
    sem_close(sem);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MarvelmindNavigation::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO_STREAM(get_logger(),std::fixed << "On configure at " << this->now().seconds());

    sem = sem_open(DATA_INPUT_SEMAPHORE, O_CREAT, 0777, 0);

    hedgeReceivePrepare();
    createPublishers();
    setMessageDefaults();

    timer_ = this->create_wall_timer(std::chrono::duration<double>(0.005)
                                     ,std::bind(&MarvelmindNavigation::main_loop, this));

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MarvelmindNavigation::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO_STREAM(get_logger(),std::fixed << "On shutdown at " << this->now().seconds());
    deactivateAllPublishers();
    timer_.reset();
    if (hedge != NULL)
    {
        stopMarvelmindHedge (hedge);
        destroyMarvelmindHedge (hedge);
    }
    sem_close(sem);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

int MarvelmindNavigation::hedgeReceivePrepare()
{
    // get port name from command line arguments (if specified)
    const char * ttyFileName;
    uint32_t baudRate;
    if (argc_>=2) ttyFileName=argv_[1];
    else ttyFileName=DEFAULT_TTY_FILENAME;
    if (argc_>=3) baudRate= atoi(argv_[2]);
    else baudRate=DEFAULT_TTY_BAUDRATE;

    // Init
    hedge=createMarvelmindHedge ();
    if (hedge==NULL)
    {
        RCLCPP_INFO(get_logger(),"Error: Unable to create MarvelmindHedge");
        return -1;
    }

    hedge->ttyFileName=ttyFileName;
    hedge->baudRate= baudRate;
    hedge->verbose=true; // show errors and warnings
    hedge->anyInputPacketCallback= semCallback;

    RCLCPP_INFO_STREAM(get_logger(),"Creating a connection at: " << argv_[1] << " with baud rate: " << argv_[2]);
    startMarvelmindHedge(hedge);
    return 1;
}

bool MarvelmindNavigation::hedgeReceiveCheck()
{
    if (hedge->haveNewValues_)
    {
        struct PositionValue position;
        getPositionFromMarvelmindHedge (hedge, &position);

        hedge_pos_msg.address= position.address;
        hedge_pos_ang_msg.address= position.address;

        hedge_pos_msg.flags= position.flags;
        hedge_pos_noaddress_msg.flags= position.flags;
        hedge_pos_ang_msg.flags= position.flags;
        if (hedge_pos_msg.flags&(1<<1))// flag of timestamp format
        {
            hedge_pos_msg.timestamp_ms= position.timestamp;// msec
            hedge_pos_noaddress_msg.timestamp_ms= position.timestamp;
        }
        else
        {
            hedge_pos_msg.timestamp_ms= position.timestamp*15.625;// alpha-cycles ==> msec
            hedge_pos_noaddress_msg.timestamp_ms= position.timestamp*15.625;
        }
        hedge_pos_ang_msg.timestamp_ms= position.timestamp;

        hedge_pos_msg.x_m= position.x/1000.0;
        hedge_pos_msg.y_m= position.y/1000.0;
        hedge_pos_msg.z_m= position.z/1000.0;

        hedge_pos_noaddress_msg.x_m= position.x/1000.0;
        hedge_pos_noaddress_msg.y_m= position.y/1000.0;
        hedge_pos_noaddress_msg.z_m= position.z/1000.0;

        hedge_pos_ang_msg.x_m= position.x/1000.0;
        hedge_pos_ang_msg.y_m= position.y/1000.0;
        hedge_pos_ang_msg.z_m= position.z/1000.0;

        hedge_pos_ang_msg.angle= position.angle;

        hedge->haveNewValues_=false;

        return true;
    }
    return false;
}

bool MarvelmindNavigation::beaconReceiveCheck()
{
    uint8_t i;
    struct StationaryBeaconsPositions positions;
    struct StationaryBeaconPosition *bp= NULL;
    bool foundUpd= false;
    uint8_t n;

    getStationaryBeaconsPositionsFromMarvelmindHedge (hedge, &positions);
    n= positions.numBeacons;
    if (n == 0)
        return false;

    for(i=0;i<n;i++)
    {
        bp= &positions.beacons[i];
        if (bp->updatedForMsg)
        {
            clearStationaryBeaconUpdatedFlag(hedge, bp->address);
            foundUpd= true;
            break;
        }
    }
    if (!foundUpd)
        return false;
    if (bp == NULL)
        return false;

    beacon_pos_msg.address= bp->address;
    beacon_pos_msg.x_m= bp->x/1000.0;
    beacon_pos_msg.y_m= bp->y/1000.0;
    beacon_pos_msg.z_m= bp->z/1000.0;

    return true;
}

bool MarvelmindNavigation::hedgeIMURawReceiveCheck()
{
    if (!hedge->rawIMU.updated)
        return false;

    hedge_imu_raw_msg.acc_x= hedge->rawIMU.acc_x;
    hedge_imu_raw_msg.acc_y= hedge->rawIMU.acc_y;
    hedge_imu_raw_msg.acc_z= hedge->rawIMU.acc_z;

    hedge_imu_raw_msg.gyro_x= hedge->rawIMU.gyro_x;
    hedge_imu_raw_msg.gyro_y= hedge->rawIMU.gyro_y;
    hedge_imu_raw_msg.gyro_z= hedge->rawIMU.gyro_z;

    hedge_imu_raw_msg.compass_x= hedge->rawIMU.compass_x;
    hedge_imu_raw_msg.compass_y= hedge->rawIMU.compass_y;
    hedge_imu_raw_msg.compass_z= hedge->rawIMU.compass_z;

    hedge_imu_raw_msg.timestamp_ms= hedge->rawIMU.timestamp;

    hedge->rawIMU.updated= false;

    return true;

}

bool MarvelmindNavigation::hedgeIMUFusionReceiveCheck()
{
    if (!hedge->fusionIMU.updated)
        return false;

    hedge_imu_fusion_msg.x_m= hedge->fusionIMU.x/1000.0;
    hedge_imu_fusion_msg.y_m= hedge->fusionIMU.y/1000.0;
    hedge_imu_fusion_msg.z_m= hedge->fusionIMU.z/1000.0;

    hedge_imu_fusion_msg.qw= hedge->fusionIMU.qw/10000.0;
    hedge_imu_fusion_msg.qx= hedge->fusionIMU.qx/10000.0;
    hedge_imu_fusion_msg.qy= hedge->fusionIMU.qy/10000.0;
    hedge_imu_fusion_msg.qz= hedge->fusionIMU.qz/10000.0;

    hedge_imu_fusion_msg.vx= hedge->fusionIMU.vx/1000.0;
    hedge_imu_fusion_msg.vy= hedge->fusionIMU.vy/1000.0;
    hedge_imu_fusion_msg.vz= hedge->fusionIMU.vz/1000.0;

    hedge_imu_fusion_msg.ax= hedge->fusionIMU.ax/1000.0;
    hedge_imu_fusion_msg.ay= hedge->fusionIMU.ay/1000.0;
    hedge_imu_fusion_msg.az= hedge->fusionIMU.az/1000.0;

    hedge_imu_fusion_msg.timestamp_ms= hedge->fusionIMU.timestamp;

    hedge->fusionIMU.updated= false;

    return true;
}

void MarvelmindNavigation::getRawDistance(uint8_t index)
{
    beacon_raw_distance_msg.address_hedge= hedge->rawDistances.address_hedge;
    beacon_raw_distance_msg.address_beacon= hedge->rawDistances.distances[index].address_beacon;
    beacon_raw_distance_msg.distance_m= hedge->rawDistances.distances[index].distance/1000.0;
}

bool MarvelmindNavigation::hedgeTelemetryUpdateCheck()
{
    if (!hedge->telemetry.updated)
        return false;

    hedge_telemetry_msg.battery_voltage= hedge->telemetry.vbat_mv/1000.0;
    hedge_telemetry_msg.rssi_dbm= hedge->telemetry.rssi_dbm;

    hedge->telemetry.updated= false;
    return true;
}

bool MarvelmindNavigation::hedgeQualityUpdateCheck()
{
    if (!hedge->quality.updated)
        return false;

    hedge_quality_msg.address= hedge->quality.address;
    hedge_quality_msg.quality_percents= hedge->quality.quality_per;

    hedge->quality.updated= false;
    return true;
}

bool MarvelmindNavigation::marvelmindWaypointUpdateCheck()
{
    uint8_t i,n;
    uint8_t nUpdated;

    if (!hedge->waypoints.updated)
        return false;

    nUpdated= 0;
    n= hedge->waypoints.numItems;
    for(i=0;i<n;i++)
    {
        if (!hedge->waypoints.items[i].updated)
            continue;

        nUpdated++;
        if (nUpdated == 1)
        {
            marvelmind_waypoint_msg.total_items= n;
            marvelmind_waypoint_msg.item_index= i;

            marvelmind_waypoint_msg.movement_type= hedge->waypoints.items[i].movementType;
            marvelmind_waypoint_msg.param1= hedge->waypoints.items[i].param1;
            marvelmind_waypoint_msg.param2= hedge->waypoints.items[i].param2;
            marvelmind_waypoint_msg.param3= hedge->waypoints.items[i].param3;

            hedge->waypoints.items[i].updated= false;
        }
    }

    if (nUpdated==1)
    {
        hedge->waypoints.updated= false;
    }
    return (nUpdated>0);
}

void MarvelmindNavigation::activateAllPublishers()
{
    hedge_pos_ang_publisher_->on_activate();
    hedge_pos_publisher_->on_activate();
    hedge_pos_noaddress_publisher_->on_activate();
    beacons_pos_publisher_->on_activate();
    hedge_imu_raw_publisher_->on_activate();
    hedge_imu_fusion_publisher_->on_activate();
    beacon_distance_publisher_->on_activate();
    hedge_telemetry_publisher_->on_activate();
    hedge_quality_publisher_->on_activate();
    marvelmind_waypoint_publisher_->on_activate();
    are_publishers_active_= true;
}

void MarvelmindNavigation::deactivateAllPublishers()
{
    hedge_pos_ang_publisher_->on_deactivate();
    hedge_pos_publisher_->on_deactivate();
    hedge_pos_noaddress_publisher_->on_deactivate();
    beacons_pos_publisher_->on_deactivate();
    hedge_imu_raw_publisher_->on_deactivate();
    hedge_imu_fusion_publisher_->on_deactivate();
    beacon_distance_publisher_->on_deactivate();
    hedge_telemetry_publisher_->on_deactivate();
    hedge_quality_publisher_->on_deactivate();
    marvelmind_waypoint_publisher_->on_deactivate();
    are_publishers_active_ = false;
}

void MarvelmindNavigation::resetAllPublishers()
{
    hedge_pos_ang_publisher_.reset();
    hedge_pos_publisher_.reset();
    hedge_pos_noaddress_publisher_.reset();
    beacons_pos_publisher_.reset();
    hedge_imu_raw_publisher_.reset();
    hedge_imu_fusion_publisher_.reset();
    beacon_distance_publisher_.reset();
    hedge_telemetry_publisher_.reset();
    hedge_quality_publisher_.reset();
    marvelmind_waypoint_publisher_.reset();
    are_publishers_active_ = false;
}

void MarvelmindNavigation::setMessageDefaults()
{
    // default values for position message
    hedge_pos_ang_msg.address= 0;
    hedge_pos_ang_msg.timestamp_ms = 0;
    hedge_pos_ang_msg.x_m = 0.0;
    hedge_pos_ang_msg.y_m = 0.0;
    hedge_pos_ang_msg.z_m = 0.0;
    hedge_pos_ang_msg.flags = (1<<0);// 'data not available' flag
    hedge_pos_ang_msg.angle= 0.0;

    hedge_pos_msg.address= 0;
    hedge_pos_msg.timestamp_ms = 0;
    hedge_pos_msg.x_m = 0.0;
    hedge_pos_msg.y_m = 0.0;
    hedge_pos_msg.z_m = 0.0;
    hedge_pos_msg.flags = (1<<0);// 'data not available' flag

    hedge_pos_noaddress_msg.timestamp_ms = 0;
    hedge_pos_noaddress_msg.x_m = 0.0;
    hedge_pos_noaddress_msg.y_m = 0.0;
    hedge_pos_noaddress_msg.z_m = 0.0;
    hedge_pos_noaddress_msg.flags = (1<<0);// 'data not available' flag

    beacon_pos_msg.address= 0;
    beacon_pos_msg.x_m = 0.0;
    beacon_pos_msg.y_m = 0.0;
    beacon_pos_msg.z_m = 0.0;
}

void MarvelmindNavigation::createPublishers()
{
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));

    hedge_pos_ang_publisher_ = this->create_publisher<marvelmind_interfaces::msg::HedgePosAng>(HEDGE_POSITION_WITH_ANGLE_TOPIC_NAME, qos);
    hedge_pos_publisher_ = this->create_publisher<marvelmind_interfaces::msg::HedgePosA>(HEDGE_POSITION_ADDRESSED_TOPIC_NAME, qos);
    hedge_pos_noaddress_publisher_ = this->create_publisher<marvelmind_interfaces::msg::HedgePos>(HEDGE_POSITION_TOPIC_NAME, qos);

    beacons_pos_publisher_ = this->create_publisher<marvelmind_interfaces::msg::BeaconPosA>(BEACONS_POSITION_ADDRESSED_TOPIC_NAME, qos);

    hedge_imu_raw_publisher_ = this->create_publisher<marvelmind_interfaces::msg::HedgeImuRaw>(HEDGE_IMU_RAW_TOPIC_NAME, qos);
    hedge_imu_fusion_publisher_ = this->create_publisher<marvelmind_interfaces::msg::HedgeImuFusion>(HEDGE_IMU_FUSION_TOPIC_NAME, qos);

    beacon_distance_publisher_ = this->create_publisher<marvelmind_interfaces::msg::BeaconDistance>(BEACON_RAW_DISTANCE_TOPIC_NAME, qos);

    hedge_telemetry_publisher_ = this->create_publisher<marvelmind_interfaces::msg::HedgeTelemetry>(HEDGE_TELEMETRY_TOPIC_NAME, qos);
    hedge_quality_publisher_ = this->create_publisher<marvelmind_interfaces::msg::HedgeQuality>(HEDGE_QUALITY_TOPIC_NAME, qos);

    marvelmind_waypoint_publisher_ = this->create_publisher<marvelmind_interfaces::msg::MarvelmindWaypoint>(MARVELMIND_WAYPOINT_TOPIC_NAME, qos);
}

void MarvelmindNavigation::main_loop()
{
    //    RCLCPP_INFO_STREAM(get_logger(),std::fixed << "Running main loop at " << this->now().seconds() );

    if (hedge->terminationRequired)
    {
        RCLCPP_INFO_STREAM(get_logger(),std::fixed << "Shutdown called from hedge->terminationRequired at " << now().seconds());
        this->shutdown();
    }

    if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
    {
        RCLCPP_INFO_STREAM(get_logger(),std::fixed << "clock_gettime error. Realtime: " << CLOCK_REALTIME << " vs ts: " << ts.tv_sec);
        return;
    }
    ts.tv_sec += 2;
    sem_timedwait(sem,&ts);

    if (hedgeReceiveCheck())
    {
        // hedgehog data received
        RCLCPP_INFO(get_logger(), "Address: %d, timestamp: %d, %d, X=%.3f  Y= %.3f  Z=%.3f  Angle: %.1f  flags=%d",
                    (int) hedge_pos_ang_msg.address,
                    (int) hedge_pos_ang_msg.timestamp_ms,
                    (int) (hedge_pos_ang_msg.timestamp_ms - hedge_timestamp_prev),
                    (float) hedge_pos_ang_msg.x_m, (float) hedge_pos_ang_msg.y_m, (float) hedge_pos_ang_msg.z_m,
                    (float) hedge_pos_ang_msg.angle,
                    (int) hedge_pos_msg.flags);
        if(are_publishers_active_)
        {
            hedge_pos_ang_publisher_->publish(hedge_pos_ang_msg);
            hedge_pos_publisher_->publish(hedge_pos_msg);
            hedge_pos_noaddress_publisher_->publish(hedge_pos_noaddress_msg);
        }

        hedge_timestamp_prev= hedge_pos_ang_msg.timestamp_ms;
    }

    beaconReadIterations= 0;
    while(beaconReceiveCheck())
    {// stationary beacons data received
        RCLCPP_INFO(get_logger(), "Stationary beacon: Address: %d, X=%.3f  Y= %.3f  Z=%.3f",
                    (int) beacon_pos_msg.address,
                    (float) beacon_pos_msg.x_m, (float) beacon_pos_msg.y_m, (float) beacon_pos_msg.z_m);
        if(are_publishers_active_)
        {
            beacons_pos_publisher_->publish(beacon_pos_msg);
        }

        if ((beaconReadIterations++)>4)
            break;
    }

    if (hedgeIMURawReceiveCheck())
    {
        RCLCPP_INFO(get_logger(), "Raw IMU: Timestamp: %08d, aX=%05d aY=%05d aZ=%05d  gX=%05d gY=%05d gZ=%05d  cX=%05d cY=%05d cZ=%05d",
                    (int) hedge_imu_raw_msg.timestamp_ms,
                    (int) hedge_imu_raw_msg.acc_x, (int) hedge_imu_raw_msg.acc_y, (int) hedge_imu_raw_msg.acc_z,
                    (int) hedge_imu_raw_msg.gyro_x, (int) hedge_imu_raw_msg.gyro_y, (int) hedge_imu_raw_msg.gyro_z,
                    (int) hedge_imu_raw_msg.compass_x, (int) hedge_imu_raw_msg.compass_y, (int) hedge_imu_raw_msg.compass_z);
        if(are_publishers_active_)
        {
            hedge_imu_raw_publisher_->publish(hedge_imu_raw_msg);
        }
    }

    if (hedgeIMUFusionReceiveCheck())
    {
        RCLCPP_INFO(get_logger(), "IMU fusion: Timestamp: %08d, X=%.3f  Y= %.3f  Z=%.3f  q=%.3f,%.3f,%.3f,%.3f v=%.3f,%.3f,%.3f  a=%.3f,%.3f,%.3f",
                    (int) hedge_imu_fusion_msg.timestamp_ms,
                    (float) hedge_imu_fusion_msg.x_m, (float) hedge_imu_fusion_msg.y_m, (float) hedge_imu_fusion_msg.z_m,
                    (float) hedge_imu_fusion_msg.qw, (float) hedge_imu_fusion_msg.qx, (float) hedge_imu_fusion_msg.qy, (float) hedge_imu_fusion_msg.qz,
                    (float) hedge_imu_fusion_msg.vx, (float) hedge_imu_fusion_msg.vy, (float) hedge_imu_fusion_msg.vz,
                    (float) hedge_imu_fusion_msg.ax, (float) hedge_imu_fusion_msg.ay, (float) hedge_imu_fusion_msg.az);
        if(are_publishers_active_)
        {
            hedge_imu_fusion_publisher_->publish(hedge_imu_fusion_msg);
        }
    }

    if (hedge->rawDistances.updated)
    {
        uint8_t i;
        for(i=0;i<4;i++)
        {
            getRawDistance(i);
            if (beacon_raw_distance_msg.address_beacon != 0)
            {
                RCLCPP_INFO(get_logger(), "Raw distance: %02d ==> %02d,  Distance= %.3f ",
                            (int) beacon_raw_distance_msg.address_hedge,
                            (int) beacon_raw_distance_msg.address_beacon,
                            (float) beacon_raw_distance_msg.distance_m);
                if(are_publishers_active_)
                {
                    beacon_distance_publisher_->publish(beacon_raw_distance_msg);
                }
            }
        }
        hedge->rawDistances.updated= false;
    }

    if (hedgeTelemetryUpdateCheck())
    {
        RCLCPP_INFO(get_logger(), "Vbat= %.3f V, RSSI= %02d ",
                    (float) hedge_telemetry_msg.battery_voltage,
                    (int) hedge_telemetry_msg.rssi_dbm);
        if(are_publishers_active_)
            hedge_telemetry_publisher_->publish(hedge_telemetry_msg);
    }

    if (hedgeQualityUpdateCheck())
    {
        RCLCPP_INFO(get_logger(), "Quality: Address= %d,  Quality= %02d %% ",
                    (int) hedge_quality_msg.address,
                    (int) hedge_quality_msg.quality_percents);
        if(are_publishers_active_)
            hedge_quality_publisher_->publish(hedge_quality_msg);
    }

    if (marvelmindWaypointUpdateCheck())
    {
        int n= marvelmind_waypoint_msg.item_index+1;
        RCLCPP_INFO(get_logger(), "Waypoint %03d/%03d: Type= %03d,  Param1= %05d, Param2= %05d, Param3= %05d ",
                    (int) n,
                    (int) marvelmind_waypoint_msg.total_items, marvelmind_waypoint_msg.movement_type,
                    marvelmind_waypoint_msg.param1, marvelmind_waypoint_msg.param2, marvelmind_waypoint_msg.param3);
        if(are_publishers_active_)
            marvelmind_waypoint_publisher_->publish(marvelmind_waypoint_msg);
    }

}


int main(int argc, char * argv[])
{
    // force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<MarvelmindNavigation> lc_node = std::make_shared<MarvelmindNavigation>("lc_marvel2", argc, argv);

    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}

