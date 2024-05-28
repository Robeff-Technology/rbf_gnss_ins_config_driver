#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <rbf_gnss_ins_config_driver/serial_port.h>
#include <rbf_gnss_ins_config_driver/srv/change_baud.hpp>
#include <rbf_gnss_ins_config_driver/srv/reset.hpp>
#include <rbf_gnss_ins_config_driver/srv/get_config.hpp>
#include <rbf_gnss_ins_config_driver/srv/set_config.hpp>
#include <rbf_gnss_ins_config_driver/srv/write_command.hpp>


namespace rbf_gnss_ins_config_driver{

class ConfigGnssIns: public rclcpp::Node {
    private:
    void create_msg_period(const std::string& port, std::string msg, float period );
    std::string extract_substring(const char* input, char startChar, char endChar);
    std::string config_response_; 
    struct Config{
        struct Serial{
            std::string port_name;
            int baudrate;
        };

        struct PortConfig{
            double rawimu_period; 
            double inspvax_period;
            double uniheading_period;
            double bestgnsspos_period;
            double bestgnssvel_period;
            double ecef_period;
            bool gprmc;
        };

        struct InsConfig{
            bool enable;
            int timeout;
            double align_velocity_threshold;
            std::vector<std::string> ins_initial_attitude;
            std::vector<std::string> ins_initial_attitude_error;
            std::vector<std::string> lever_arm_master;
            std::vector<std::string> lever_arm_slave;
            std::vector<std::string> lever_arm_master_error;
            std::vector<std::string> lever_arm_slave_error;
            std::vector<std::string> imu_position_offset;
        };

        struct PPSConfig{
            bool enable;
            int mode;
            std::string polarity;
            int pulse_width;
            int period;
        };

        Serial serial;
        PortConfig port_1_config;
        PortConfig port_2_config;
        PortConfig port_3_config;
        InsConfig ins_config;
        PPSConfig pps_config;
    };
    Config config_;
    std::vector<std::string> commands_;
    std::shared_ptr<SerialPort> serial_port_ptr_;
    /*
    @brief Services
    */
    rclcpp::Service<rbf_gnss_ins_config_driver::srv::ChangeBaud>::SharedPtr change_baud_service_ptr_;
    rclcpp::Service<rbf_gnss_ins_config_driver::srv::Reset>::SharedPtr reset_service_ptr_;
    rclcpp::Service<rbf_gnss_ins_config_driver::srv::GetConfig>::SharedPtr get_config_service_ptr_;
    rclcpp::Service<rbf_gnss_ins_config_driver::srv::SetConfig>::SharedPtr set_config_service_ptr_;
    rclcpp::Service<rbf_gnss_ins_config_driver::srv::WriteCommand>::SharedPtr write_command_service_ptr_;

    public:
    ConfigGnssIns(const rclcpp::NodeOptions &options);
    ~ConfigGnssIns() override {
        if(serial_port_ptr_){
            serial_port_ptr_->close();
        }
    }

    /*
    * @brief Service callback to change baudrate of the GNSS-INS device
    */
    void change_baud_cb(const std::shared_ptr<rbf_gnss_ins_config_driver::srv::ChangeBaud::Request> request,
                        std::shared_ptr<rbf_gnss_ins_config_driver::srv::ChangeBaud::Response> response);
                        
    void reset_cb(const std::shared_ptr<rbf_gnss_ins_config_driver::srv::Reset::Request> request,
                        std::shared_ptr<rbf_gnss_ins_config_driver::srv::Reset::Response> response);

    void get_config_cb(const std::shared_ptr<rbf_gnss_ins_config_driver::srv::GetConfig::Request> request,
                       std::shared_ptr<rbf_gnss_ins_config_driver::srv::GetConfig::Response> response);
    
    void set_config_cb(const std::shared_ptr<rbf_gnss_ins_config_driver::srv::SetConfig::Request> request,
                        std::shared_ptr<rbf_gnss_ins_config_driver::srv::SetConfig::Response> response);

    void write_command_cb(const std::shared_ptr<rbf_gnss_ins_config_driver::srv::WriteCommand::Request> request,
                          std::shared_ptr<rbf_gnss_ins_config_driver::srv::WriteCommand::Response> response);

    void load_parameters();
    void convert_params_to_commands();
    void init_services();


};
}; // namespace rbf_gnss_ins_config_driver