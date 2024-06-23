#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <rbf_gnss_ins_config_driver/serial_port.h>
#include <rbf_gnss_ins_config_driver/srv/change_baud.hpp>
#include <rbf_gnss_ins_config_driver/srv/reset.hpp>
#include <rbf_gnss_ins_config_driver/srv/get_config.hpp>
#include <rbf_gnss_ins_config_driver/srv/set_config.hpp>
#include <rbf_gnss_ins_config_driver/srv/write_command.hpp>

namespace rbf_gnss_ins_config_driver {

/**
 * @class ConfigGnssIns
 * @brief This class provides functionalities to configure the GNSS-INS device.
 */
class ConfigGnssIns : public rclcpp::Node {
private:
    /**
     * @brief Creates a message period configuration for a given port.
     * @param port The name of the port.
     * @param msg The message to configure.
     * @param period The period for the message.
     */
    void create_msg_period(const std::string& port, std::string msg, float period);

    /**
     * @brief Extracts a substring from input between startChar and endChar.
     * @param input The input string.
     * @param startChar The character to start extraction.
     * @param endChar The character to end extraction.
     * @return The extracted substring.
     */
    std::string extract_substring(const char* input, char startChar, char endChar);

    /// The configuration response.
    std::string config_response_; 

    /**
     * @struct Config
     * @brief This struct holds the configuration settings for the GNSS-INS device.
     */
    struct Config {
        /**
         * @struct Serial
         * @brief Serial port configuration.
         */
        struct Serial {
            std::string port_name; ///< Name of the serial port.
            int baudrate; ///< Baudrate of the serial port.
        };

        /**
         * @struct PortConfig
         * @brief Configuration for the ports.
         */
        struct PortConfig {
            double rawimu_period; ///< Period for raw IMU messages.
            double inspvax_period; ///< Period for inspvax messages.
            double uniheading_period; ///< Period for uniheading messages.
            double bestgnsspos_period; ///< Period for best GNSS position messages.
            double bestgnssvel_period; ///< Period for best GNSS velocity messages.
            double ecef_period; ///< Period for ECEF messages.
            bool gprmc; ///< Flag for GPRMC messages.
        };

        /**
         * @struct InsConfig
         * @brief Configuration for the INS.
         */
        struct InsConfig {
            bool enable; ///< Flag to enable INS.
            int timeout; ///< Timeout for INS.
            double align_velocity_threshold; ///< Velocity threshold for alignment.
            std::vector<std::string> ins_initial_attitude; ///< Initial attitude for INS.
            std::vector<std::string> ins_initial_attitude_error; ///< Initial attitude error for INS.
            std::vector<std::string> lever_arm_master; ///< Lever arm for master.
            std::vector<std::string> lever_arm_slave; ///< Lever arm for slave.
            std::vector<std::string> lever_arm_master_error; ///< Lever arm error for master.
            std::vector<std::string> lever_arm_slave_error; ///< Lever arm error for slave.
            std::vector<std::string> imu_position_offset; ///< IMU position offset.
        };

        /**
         * @struct PPSConfig
         * @brief Configuration for PPS.
         */
        struct PPSConfig {
            bool enable; ///< Flag to enable PPS.
            int mode; ///< Mode for PPS.
            std::string polarity; ///< Polarity for PPS.
            int pulse_width; ///< Pulse width for PPS.
            int period; ///< Period for PPS.
        };

        Serial serial; ///< Serial configuration.
        PortConfig port_1_config; ///< Configuration for port 1.
        PortConfig port_2_config; ///< Configuration for port 2.
        PortConfig port_3_config; ///< Configuration for port 3.
        InsConfig ins_config; ///< INS configuration.
        PPSConfig pps_config; ///< PPS configuration.
    };

    /// Configuration instance.
    Config config_;

    /// List of commands.
    std::vector<std::string> commands_;

    /// Shared pointer to the serial port.
    std::shared_ptr<SerialPort> serial_port_ptr_;

    /// @brief Service to change baudrate.
    rclcpp::Service<rbf_gnss_ins_config_driver::srv::ChangeBaud>::SharedPtr change_baud_service_ptr_;

    /// @brief Service to reset the device.
    rclcpp::Service<rbf_gnss_ins_config_driver::srv::Reset>::SharedPtr reset_service_ptr_;

    /// @brief Service to get the configuration.
    rclcpp::Service<rbf_gnss_ins_config_driver::srv::GetConfig>::SharedPtr get_config_service_ptr_;

    /// @brief Service to set the configuration.
    rclcpp::Service<rbf_gnss_ins_config_driver::srv::SetConfig>::SharedPtr set_config_service_ptr_;

    /// @brief Service to write a command.
    rclcpp::Service<rbf_gnss_ins_config_driver::srv::WriteCommand>::SharedPtr write_command_service_ptr_;

public:
    /**
     * @brief Constructor for the ConfigGnssIns class.
     * @param options Node options.
     */
    ConfigGnssIns(const rclcpp::NodeOptions &options);

    /**
     * @brief Destructor for the ConfigGnssIns class.
     */
    ~ConfigGnssIns() override {
        if(serial_port_ptr_) {
            serial_port_ptr_->close();
        }
    }

    /**
     * @brief Service callback to change baudrate of the GNSS-INS device.
     * @param request The service request.
     * @param response The service response.
     */
    void change_baud_cb(const std::shared_ptr<rbf_gnss_ins_config_driver::srv::ChangeBaud::Request> request,
                        std::shared_ptr<rbf_gnss_ins_config_driver::srv::ChangeBaud::Response> response);

    /**
     * @brief Service callback to reset the GNSS-INS device.
     * @param request The service request.
     * @param response The service response.
     */
    void reset_cb(const std::shared_ptr<rbf_gnss_ins_config_driver::srv::Reset::Request> request,
                  std::shared_ptr<rbf_gnss_ins_config_driver::srv::Reset::Response> response);

    /**
     * @brief Service callback to get the configuration of the GNSS-INS device.
     * @param request The service request.
     * @param response The service response.
     */
    void get_config_cb(const std::shared_ptr<rbf_gnss_ins_config_driver::srv::GetConfig::Request> request,
                       std::shared_ptr<rbf_gnss_ins_config_driver::srv::GetConfig::Response> response);

    /**
     * @brief Service callback to set the configuration of the GNSS-INS device.
     * @param request The service request.
     * @param response The service response.
     */
    void set_config_cb(const std::shared_ptr<rbf_gnss_ins_config_driver::srv::SetConfig::Request> request,
                       std::shared_ptr<rbf_gnss_ins_config_driver::srv::SetConfig::Response> response);

    /**
     * @brief Service callback to write a command to the GNSS-INS device.
     * @param request The service request.
     * @param response The service response.
     */
    void write_command_cb(const std::shared_ptr<rbf_gnss_ins_config_driver::srv::WriteCommand::Request> request,
                          std::shared_ptr<rbf_gnss_ins_config_driver::srv::WriteCommand::Response> response);

    /**
     * @brief Loads parameters from the parameter server.
     */
    void load_parameters();

    /**
     * @brief Converts loaded parameters to commands.
     */
    void convert_params_to_commands();

    /**
     * @brief Initializes the services provided by this node.
     */
    void init_services();
};

} // namespace rbf_gnss_ins_config_driver
