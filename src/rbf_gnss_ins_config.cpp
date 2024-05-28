#include <rbf_gnss_ins_config_driver/rbf_gnss_ins_config.h>
#include <boost/algorithm/string.hpp>

namespace rbf_gnss_ins_config_driver{

    ConfigGnssIns::ConfigGnssIns(const rclcpp::NodeOptions &options): Node("rbf_gnss_ins_config_driver", options){
        load_parameters();
        convert_params_to_commands();
        try {
            serial_port_ptr_ = std::make_shared<SerialPort>(config_.serial.port_name.c_str());
            serial_port_ptr_->open();
            serial_port_ptr_->configure(config_.serial.baudrate, 8, 'N', 1);
        }
        catch (const SerialPortException& e){
            RCLCPP_ERROR(get_logger(), e.what());
            rclcpp::shutdown();
        }
        init_services();
    }

    void ConfigGnssIns::load_parameters(){
        config_.serial.port_name = declare_parameter<std::string>("serial_config.port", "/dev/ttyUSB0");
        config_.serial.baudrate = declare_parameter<int>("serial_config.baudrate", 460800);

        config_.port_1_config.rawimu_period = declare_parameter<double>("port1_config.rawimu_period", 0.01);
        config_.port_1_config.inspvax_period = declare_parameter<double>("port1_config.inspvax_period", 0.01);
        config_.port_1_config.uniheading_period = declare_parameter<double>("port1_config.uniheading_period", 0.01);
        config_.port_1_config.bestgnsspos_period = declare_parameter<double>("port1_config.bestgnsspos_period", 0.01);
        config_.port_1_config.bestgnssvel_period = declare_parameter<double>("port1_config.bestgnssvel_period", 0.01);
        config_.port_1_config.ecef_period = declare_parameter<double>("port1_config.ecef_period", 0.01);
        config_.port_1_config.gprmc = declare_parameter<bool>("port1_config.gprmc", false);

        config_.port_2_config.rawimu_period = declare_parameter<double>("port2_config.rawimu_period", 0.01);
        config_.port_2_config.inspvax_period = declare_parameter<double>("port2_config.inspvax_period", 0.01);
        config_.port_2_config.uniheading_period = declare_parameter<double>("port2_config.uniheading_period", 0.01);
        config_.port_2_config.bestgnsspos_period = declare_parameter<double>("port2_config.bestgnsspos_period", 0.01);
        config_.port_2_config.bestgnssvel_period = declare_parameter<double>("port2_config.bestgnssvel_period", 0.01);
        config_.port_2_config.ecef_period = declare_parameter<double>("port2_config.ecef_period", 0.01);
        config_.port_2_config.gprmc = declare_parameter<bool>("port2_config.gprmc", false);

        config_.port_3_config.rawimu_period = declare_parameter<double>("port3_config.rawimu_period", 0.01);
        config_.port_3_config.inspvax_period = declare_parameter<double>("port3_config.inspvax_period", 0.01);
        config_.port_3_config.uniheading_period = declare_parameter<double>("port3_config.uniheading_period", 0.01);
        config_.port_3_config.bestgnsspos_period = declare_parameter<double>("port3_config.bestgnsspos_period", 0.01);
        config_.port_3_config.bestgnssvel_period = declare_parameter<double>("port3_config.bestgnssvel_period", 0.01);
        config_.port_3_config.ecef_period = declare_parameter<double>("port3_config.ecef_period", 0.01);
        config_.port_3_config.gprmc = declare_parameter<bool>("port3_config.gprmc", false);
        
        config_.ins_config.enable = declare_parameter<bool>("ins_config.enable", false);
        config_.ins_config.timeout = declare_parameter<int>("ins_config.timeout", 10);
        config_.ins_config.align_velocity_threshold = declare_parameter<double>("ins_config.align_velocity_threshold", 0.1);
        config_.ins_config.ins_initial_attitude = declare_parameter<std::vector<std::string>>("ins_config.ins_initial_attitude", {"0.0", "0.0", "0.0"});
        config_.ins_config.ins_initial_attitude_error = declare_parameter<std::vector<std::string>>("ins_config.ins_initial_attitude_error", {"0.0", "0.0", "0.0"});
        config_.ins_config.lever_arm_master = declare_parameter<std::vector<std::string>>("ins_config.lever_arm_master", {"0.0", "0.0", "0.0"});
        config_.ins_config.lever_arm_slave = declare_parameter<std::vector<std::string>>("ins_config.lever_arm_slave", {"0.0", "0.0", "0.0"});
        config_.ins_config.lever_arm_master_error = declare_parameter<std::vector<std::string>>("ins_config.lever_arm_master_error", {"0.0", "0.0", "0.0"});
        config_.ins_config.lever_arm_slave_error = declare_parameter<std::vector<std::string>>("ins_config.lever_arm_slave_error", {"0.0", "0.0", "0.0"});
        config_.ins_config.imu_position_offset = declare_parameter<std::vector<std::string>>("ins_config.imu_position_offset", {"0.0", "0.0", "0.0"});


        
        boost::split(config_.ins_config.ins_initial_attitude, config_.ins_config.ins_initial_attitude[0], boost::is_any_of(" "));
        boost::split(config_.ins_config.ins_initial_attitude_error, config_.ins_config.ins_initial_attitude_error[0], boost::is_any_of(" "));
        boost::split(config_.ins_config.lever_arm_master, config_.ins_config.lever_arm_master[0], boost::is_any_of(" "));
        boost::split(config_.ins_config.lever_arm_slave, config_.ins_config.lever_arm_slave[0], boost::is_any_of(" "));
        boost::split(config_.ins_config.lever_arm_master_error, config_.ins_config.lever_arm_master_error[0], boost::is_any_of(" "));
        boost::split(config_.ins_config.lever_arm_slave_error, config_.ins_config.lever_arm_slave_error[0], boost::is_any_of(" "));
        boost::split(config_.ins_config.imu_position_offset, config_.ins_config.imu_position_offset[0], boost::is_any_of(" "));

        config_.pps_config.enable = declare_parameter<bool>("pps_config.enable", false);
        config_.pps_config.mode = declare_parameter<int>("pps_config.mode", 0);
        config_.pps_config.polarity = declare_parameter<std::string>("pps_config.polarity", "POSITIVE");
        config_.pps_config.pulse_width = declare_parameter<int>("pps_config.width", 500000);
        config_.pps_config.period = declare_parameter<int>("pps_config.period", 1000);

        RCLCPP_INFO(get_logger(), "---------------- PARAMETERS ----------------");
        RCLCPP_INFO(get_logger(), "Serial Port: %s", config_.serial.port_name.c_str());
        RCLCPP_INFO(get_logger(), "Baudrate: %d", config_.serial.baudrate);
        RCLCPP_INFO(get_logger(), "-------------------------------------------");
        RCLCPP_INFO(get_logger(), "Port 1 Config:");
        RCLCPP_INFO(get_logger(), "RawIMU Period: %f", config_.port_1_config.rawimu_period);
        RCLCPP_INFO(get_logger(), "INSPVAX Period: %f", config_.port_1_config.inspvax_period);
        RCLCPP_INFO(get_logger(), "UNIHEADING Period: %f", config_.port_1_config.uniheading_period);
        RCLCPP_INFO(get_logger(), "BESTGNSSPOS Period: %f", config_.port_1_config.bestgnsspos_period);
        RCLCPP_INFO(get_logger(), "BESTGNSSVEL Period: %f", config_.port_1_config.bestgnssvel_period);
        RCLCPP_INFO(get_logger(), "ECEF Period: %f", config_.port_1_config.ecef_period);
        RCLCPP_INFO(get_logger(), "GPRMC: %d", config_.port_1_config.gprmc);
        RCLCPP_INFO(get_logger(), "-------------------------------------------");
        RCLCPP_INFO(get_logger(), "Port 2 Config:");
        RCLCPP_INFO(get_logger(), "RawIMU Period: %f", config_.port_2_config.rawimu_period);
        RCLCPP_INFO(get_logger(), "INSPVAX Period: %f", config_.port_2_config.inspvax_period);
        RCLCPP_INFO(get_logger(), "UNIHEADING Period: %f", config_.port_2_config.uniheading_period);
        RCLCPP_INFO(get_logger(), "BESTGNSSPOS Period: %f", config_.port_2_config.bestgnsspos_period);
        RCLCPP_INFO(get_logger(), "BESTGNSSVEL Period: %f", config_.port_2_config.bestgnssvel_period);
        RCLCPP_INFO(get_logger(), "ECEF Period: %f", config_.port_2_config.ecef_period);
        RCLCPP_INFO(get_logger(), "GPRMC: %d", config_.port_2_config.gprmc);
        RCLCPP_INFO(get_logger(), "-------------------------------------------");
        RCLCPP_INFO(get_logger(), "Port 3 Config:");
        RCLCPP_INFO(get_logger(), "RawIMU Period: %f", config_.port_3_config.rawimu_period);
        RCLCPP_INFO(get_logger(), "INSPVAX Period: %f", config_.port_3_config.inspvax_period);
        RCLCPP_INFO(get_logger(), "UNIHEADING Period: %f", config_.port_3_config.uniheading_period);
        RCLCPP_INFO(get_logger(), "BESTGNSSPOS Period: %f", config_.port_3_config.bestgnsspos_period);
        RCLCPP_INFO(get_logger(), "BESTGNSSVEL Period: %f", config_.port_3_config.bestgnssvel_period);
        RCLCPP_INFO(get_logger(), "ECEF Period: %f", config_.port_3_config.ecef_period);
        RCLCPP_INFO(get_logger(), "GPRMC: %d", config_.port_3_config.gprmc);
        RCLCPP_INFO(get_logger(), "-------------------------------------------");
        RCLCPP_INFO(get_logger(), "INS Config:");
        RCLCPP_INFO(get_logger(), "Enable: %d", config_.ins_config.enable);
        RCLCPP_INFO(get_logger(), "Timeout: %d", config_.ins_config.timeout);
        RCLCPP_INFO(get_logger(), "Align Velocity Threshold: %f", config_.ins_config.align_velocity_threshold);
        RCLCPP_INFO(get_logger(), "INS Initial Attitude: %s %s %s", config_.ins_config.ins_initial_attitude[0].c_str(), config_.ins_config.ins_initial_attitude[1].c_str(), config_.ins_config.ins_initial_attitude[2].c_str());
        RCLCPP_INFO(get_logger(), "INS Initial Attitude Error: %s %s %s", config_.ins_config.ins_initial_attitude_error[0].c_str(), config_.ins_config.ins_initial_attitude_error[1].c_str(), config_.ins_config.ins_initial_attitude_error[2].c_str());
        RCLCPP_INFO(get_logger(), "Lever Arm Master: %s %s %s", config_.ins_config.lever_arm_master[0].c_str(), config_.ins_config.lever_arm_master[1].c_str(), config_.ins_config.lever_arm_master[2].c_str());
        RCLCPP_INFO(get_logger(), "Lever Arm Slave: %s %s %s", config_.ins_config.lever_arm_slave[0].c_str(), config_.ins_config.lever_arm_slave[1].c_str(), config_.ins_config.lever_arm_slave[2].c_str());
        RCLCPP_INFO(get_logger(), "Lever Arm Master Error: %s %s %s", config_.ins_config.lever_arm_master_error[0].c_str(), config_.ins_config.lever_arm_master_error[1].c_str(), config_.ins_config.lever_arm_master_error[2].c_str());
        RCLCPP_INFO(get_logger(), "Lever Arm Slave Error: %s %s %s", config_.ins_config.lever_arm_slave_error[0].c_str(), config_.ins_config.lever_arm_slave_error[1].c_str(), config_.ins_config.lever_arm_slave_error[2].c_str());
        RCLCPP_INFO(get_logger(), "IMU Position Offset: %s %s %s", config_.ins_config.imu_position_offset[0].c_str(), config_.ins_config.imu_position_offset[1].c_str(), config_.ins_config.imu_position_offset[2].c_str());
        RCLCPP_INFO(get_logger(), "-------------------------------------------");
        RCLCPP_INFO(get_logger(), "PPS Config:");
        RCLCPP_INFO(get_logger(), "Enable: %d", config_.pps_config.enable);
        RCLCPP_INFO(get_logger(), "Mode: %d", config_.pps_config.mode);
        RCLCPP_INFO(get_logger(), "Polarity: %s", config_.pps_config.polarity.c_str());
        RCLCPP_INFO(get_logger(), "Pulse Width: %d", config_.pps_config.pulse_width);
        RCLCPP_INFO(get_logger(), "Period: %d", config_.pps_config.period);

        RCLCPP_INFO(get_logger(), "-------------------------------------------");
    }

    void ConfigGnssIns::init_services(){
        change_baud_service_ptr_ = create_service<rbf_gnss_ins_config_driver::srv::ChangeBaud>("/robins/srv/change_baudrate", std::bind(&ConfigGnssIns::change_baud_cb, this, std::placeholders::_1, std::placeholders::_2));
        reset_service_ptr_ = create_service<rbf_gnss_ins_config_driver::srv::Reset>("/robins/srv/reset", std::bind(&ConfigGnssIns::reset_cb, this, std::placeholders::_1, std::placeholders::_2));
        get_config_service_ptr_ = create_service<rbf_gnss_ins_config_driver::srv::GetConfig>("/robins/srv/get_config", std::bind(&ConfigGnssIns::get_config_cb, this, std::placeholders::_1, std::placeholders::_2));
        set_config_service_ptr_ = create_service<rbf_gnss_ins_config_driver::srv::SetConfig>("/robins/srv/set_config", std::bind(&ConfigGnssIns::set_config_cb, this, std::placeholders::_1, std::placeholders::_2));
        write_command_service_ptr_ = create_service<rbf_gnss_ins_config_driver::srv::WriteCommand>("/robins/srv/write_command", std::bind(&ConfigGnssIns::write_command_cb, this, std::placeholders::_1, std::placeholders::_2));
    }

    void ConfigGnssIns::convert_params_to_commands(){
        commands_.clear();
        commands_.push_back("unlog\r\n");
        
        std::string command;
        if(config_.pps_config.enable){
            if(config_.pps_config.mode == 0){
                command = "config pps enable2 gps " + config_.pps_config.polarity + " " + std::to_string(config_.pps_config.pulse_width) + " " + std::to_string(config_.pps_config.period) + " 0 0" + "\r\n";
            }
            else{
                command = "config pps enable gps " + config_.pps_config.polarity + " " + std::to_string(config_.pps_config.pulse_width) + " " + std::to_string(config_.pps_config.period) + " 0 0" + "\r\n";
            }
        }
        else{
            command = "config pps disable\r\n";
        }

        commands_.push_back(command);

        if(config_.ins_config.enable){
            commands_.push_back("config ins enable\r\n");
        }
        else{
            commands_.push_back("config ins disable\r\n");
        }

        command = "config ins timeout " + std::to_string(config_.ins_config.timeout) + "\r\n";
        commands_.push_back(command);

        command = "config ins alignmentvel " +std::to_string(config_.ins_config.align_velocity_threshold) + "\r\n";
        commands_.push_back(command);

        command = "config ins attitude " + config_.ins_config.ins_initial_attitude[0] + " " +
        config_.ins_config.ins_initial_attitude[1] + " " +
        config_.ins_config.ins_initial_attitude[2] + " " + 
        config_.ins_config.ins_initial_attitude_error[0] + " " +
        config_.ins_config.ins_initial_attitude_error[1] + " " +
        config_.ins_config.ins_initial_attitude_error[2] + "\r\n";
        commands_.push_back(command);
        
        command = "config imutoant offset " + config_.ins_config.lever_arm_master[0] + " " + 
        config_.ins_config.lever_arm_master[1] + " " + 
        config_.ins_config.lever_arm_master[2] + " " +
        config_.ins_config.lever_arm_master_error[0] + " " +
        config_.ins_config.lever_arm_master_error[1] + " " +
        config_.ins_config.lever_arm_master_error[2] + "\r\n"; 
        commands_.push_back(command);

        command = "config imutoant2 offset " + config_.ins_config.lever_arm_slave[0] + " " + 
        config_.ins_config.lever_arm_slave[1] + " " + 
        config_.ins_config.lever_arm_slave[2] + " " +
        config_.ins_config.lever_arm_slave_error[0] + " " +
        config_.ins_config.lever_arm_slave_error[1] + " " +
        config_.ins_config.lever_arm_slave_error[2] + "\r\n";
        commands_.push_back(command);

        command = "config inssol offset " + config_.ins_config.imu_position_offset[0] + " " + config_.ins_config.imu_position_offset[1] + " " + config_.ins_config.imu_position_offset[2] + "\r\n";
        commands_.push_back(command);

        create_msg_period("com1", "rawimub", config_.port_1_config.rawimu_period);
        create_msg_period("com1", "inspvaxb", config_.port_1_config.inspvax_period);
        create_msg_period("com1", "headingb", config_.port_1_config.uniheading_period);
        create_msg_period("com1", "bestgnssposb", config_.port_1_config.bestgnsspos_period);
        create_msg_period("com1", "bestgnssvelb", config_.port_1_config.bestgnssvel_period);
        create_msg_period("com1", "bestxyzb", config_.port_1_config.ecef_period);

        create_msg_period("com2", "rawimub", config_.port_2_config.rawimu_period);
        create_msg_period("com2", "inspvaxb", config_.port_2_config.inspvax_period);
        create_msg_period("com2", "headingb", config_.port_2_config.uniheading_period);
        create_msg_period("com2", "bestgnssposb", config_.port_2_config.bestgnsspos_period);
        create_msg_period("com2", "bestgnssvelb", config_.port_2_config.bestgnssvel_period);
        create_msg_period("com2", "bestxyzb", config_.port_2_config.ecef_period);

        create_msg_period("com3", "rawimub", config_.port_3_config.rawimu_period);
        create_msg_period("com3", "inspvaxb", config_.port_3_config.inspvax_period);
        create_msg_period("com3", "headingb", config_.port_3_config.uniheading_period);
        create_msg_period("com3", "bestgnssposb", config_.port_3_config.bestgnsspos_period);
        create_msg_period("com3", "bestgnssvelb", config_.port_3_config.bestgnssvel_period);
        create_msg_period("com3", "bestxyzb", config_.port_3_config.ecef_period);

        if(config_.port_1_config.gprmc){
            commands_.push_back("log com1 gprmc ontime 1\r\n");
        }
        if(config_.port_2_config.gprmc){
            commands_.push_back("log com2 gprmc ontime 1\r\n");
        }
        if(config_.port_3_config.gprmc){
            commands_.push_back("log com3 gprmc ontime 1\r\n");
        }

        commands_.push_back("saveconfig\r\n");
    }

    void ConfigGnssIns::create_msg_period(const std::string& port, std::string msg, float period ){

        if(period > 0.0){
            std::string command = "log " + port + " " + msg + " ontime " +  std::to_string(period) + "\r\n";
            commands_.push_back(command);
        }
    }

    std::string ConfigGnssIns::extract_substring(const char* input, char startChar, char endChar) {
        // Convert the char array to a std::string
        std::string str(input);

        // Find the position of the start character
        size_t start = str.find(startChar);
        if (start == std::string::npos) {
            return "";  // Return an empty string if the start character is not found
        }

        // Find the position of the end character
        size_t end = str.find(endChar, start);
        if (end == std::string::npos) {
            return "";  // Return an empty string if the end character is not found
        }

        // Extract the substring between the start and end characters
        return str.substr(start + 1, end - start - 1);
    }

    void ConfigGnssIns::change_baud_cb(const std::shared_ptr<rbf_gnss_ins_config_driver::srv::ChangeBaud::Request> request, std::shared_ptr<rbf_gnss_ins_config_driver::srv::ChangeBaud::Response> response){
        try{

            if (request->port < 1 || request->port > 3 || (request->baudrate < 9600 || request->baudrate > 921600)) {
                response->success = false;
                response->error_message.data = "Invalid port or baudrate specified.";
                return;
            }

            response->success = true;
            std::string command = "config com" + std::to_string(request->port) + " " + std::to_string(request->baudrate) + "\r\n";
            serial_port_ptr_->write(command.c_str(), command.size());
            if(request->current_port_changed){
                rclcpp::sleep_for(std::chrono::milliseconds(500)); // Wait for the baudrate change to take effect
                serial_port_ptr_->configure(request->baudrate, 8, 'N', 1);
                serial_port_ptr_->open();
                response->response.data = "PORT" + std::to_string(request->port) + " BAUDRATE CHANGED TO " + std::to_string(request->baudrate);
            }
            rclcpp::Time start_time = now();
            std::string response_str;
            do{
                char buffer[1024];
                int bytes_read = serial_port_ptr_->read(buffer, sizeof(buffer));
                response_str += std::string(buffer, bytes_read);
            }while((now() - start_time).seconds() < 2);
            response_str = extract_substring(response_str.c_str(), ':', '*');
            if(!response_str.empty()){
                response->response.data = "PORT" + std::to_string(request->port) + "->" + response_str;
            }

        }
        catch(const SerialPortException& e){
            response->success = false;
            response->error_message.data = e.what();
        }
    }

    void ConfigGnssIns::reset_cb(const std::shared_ptr<rbf_gnss_ins_config_driver::srv::Reset::Request> request, std::shared_ptr<rbf_gnss_ins_config_driver::srv::Reset::Response> response){
        (void)request;
        try{
            response->success = true;
            serial_port_ptr_->write("freset\r\n", 8);
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            serial_port_ptr_->configure(115200, 8, 'N', 1);  // After Reset the all ports baudrate to 115200
            serial_port_ptr_->open();
            char buffer[1024];
            rclcpp::Time start_time = now();
            std::string config_response;
            do{
                int bytes_read = serial_port_ptr_->read(buffer, sizeof(buffer));
                config_response += std::string(buffer, bytes_read);
            }while((now() - start_time).seconds() < 2); // after reset the device takes some time to boot up and respond
            std::istringstream iss(config_response);
            std::string line;
            while(std::getline(iss, line)){
                response->response.data += extract_substring(line.c_str(), '$', '*');
                response->response.data += "\n";
            }

        }
        catch(const SerialPortException& e){
            response->success = false;
            RCLCPP_ERROR(get_logger(), e.what());
            response->error_message.data = e.what();
        }
    }

    void ConfigGnssIns::get_config_cb(const std::shared_ptr<rbf_gnss_ins_config_driver::srv::GetConfig::Request> request, std::shared_ptr<rbf_gnss_ins_config_driver::srv::GetConfig::Response> response){
        (void)request;
        try{
            response->success = true;
            serial_port_ptr_->write("unlog\r\n", 7);
            serial_port_ptr_->write("config\r\n", 8);
            char buffer[1024];
            rclcpp::Time start_time = now();
            std::string config_response;
            do{
                int bytes_read = serial_port_ptr_->read(buffer, sizeof(buffer));
                config_response += std::string(buffer, bytes_read);
            }while((now() - start_time).seconds() < 2);
            std::istringstream iss(config_response);
            std::string line;
            while(std::getline(iss, line)){
                response->config.data += extract_substring(line.c_str(), '$', '*');
                response->config.data += "\n";
            }
        }
        catch(const SerialPortException& e){
            response->success = false;
            response->error_message.data = e.what();
        }
    }

    void ConfigGnssIns::set_config_cb(const std::shared_ptr<rbf_gnss_ins_config_driver::srv::SetConfig::Request> request, std::shared_ptr<rbf_gnss_ins_config_driver::srv::SetConfig::Response> response){
        (void)request;
        try{
            response->success = true;
            for(auto& command : commands_){
                serial_port_ptr_->write(command.c_str(), command.size());
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
            serial_port_ptr_->flush();
            response->response.data = "CONFIGURATION COMPLETED SUCCESSFULLY!";
        }
        catch(const SerialPortException& e){
            response->success = false;
            RCLCPP_ERROR(get_logger(), e.what());
            response->error_message.data = e.what();
        }
    }

    void ConfigGnssIns::write_command_cb(const std::shared_ptr<rbf_gnss_ins_config_driver::srv::WriteCommand::Request> request, std::shared_ptr<rbf_gnss_ins_config_driver::srv::WriteCommand::Response> response){
        try{
            response->success = true;
            serial_port_ptr_->write(request->command.data.c_str(), request->command.data.size());
            rclcpp::Time start_time = now();
            std::string response_str;
            do{
                char buffer[1024];
                int bytes_read = serial_port_ptr_->read(buffer, sizeof(buffer));
                response_str += std::string(buffer, bytes_read);
            }while((now() - start_time).seconds() < 2);
            response_str = extract_substring(response_str.c_str(), ':', '*');
            if(!response_str.empty()){
                response->response.data = response_str;
            }
        }
        catch(const SerialPortException& e){
            response->success = false;
            RCLCPP_ERROR(get_logger(), e.what());
            response->error_message.data = e.what();
        }
    }
}; // namespace rbf_gnss_ins_config_driver

// Register the node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rbf_gnss_ins_config_driver::ConfigGnssIns)