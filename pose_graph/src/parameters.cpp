#include "parameters.h"

camodocal::CameraPtr m_camera;
Eigen::Vector3d TIC;
Eigen::Matrix3d qic;
int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
int ROW;
int COL;
std::string VINS_RESULT_PATH;
int DEBUG_IMAGE;
int FAST_RELOCALIZATION;
double DOWN_SCALE;
double DOWN_SCALE_RASPBERRY;
int LOOP_CLOSURE;
int VISUALIZE_IMU_FORWARD;

template <typename T>
T readParam(rclcpp::Node &node, std::string name)
{
    T ans;
    if (node.get_parameter(name, ans))
    {
        RCLCPP_INFO_STREAM(node.get_logger(),"Loaded " << name << ": " << ans);
    }
    else
    {
        RCLCPP_INFO_STREAM(node.get_logger(),"Failed to load " << name);
        rclcpp::shutdown();
   }
    return ans;
}
oid readParameters(std::string config_file,rclcpp::Logger logger)
{
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    double camera_visual_size = fsSettings["visualize_camera_size"];
    cameraposevisual.setScale(camera_visual_size);
    cameraposevisual.setLineWidth(camera_visual_size / 10.0);


    LOOP_CLOSURE = fsSettings["loop_closure"];
    std::string IMAGE_TOPIC;
    int LOAD_PREVIOUS_POSE_GRAPH;
    if (LOOP_CLOSURE)
    {        
        DOWN_SCALE = fsSettings["down_scale_pose"];
        DOWN_SCALE_RASPBERRY = fsSettings["down_scale_raspberry"];

        ROW = fsSettings["image_height"];
        COL = fsSettings["image_width"];

        ROW = ROW / DOWN_SCALE;
        COL = COL / DOWN_SCALE;

        std::string pkg_path = ros::package::getPath("pose_graph");
        std::string vocabulary_file = pkg_path + "/../support_files/brief_k10L6.bin";
        std::cout << "vocabulary_file" << vocabulary_file << std::endl;
        posegraph.loadVocabulary(vocabulary_file);

        BRIEF_PATTERN_FILE = pkg_path + "/../support_files/brief_pattern.yml";
        std::cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << std::endl;
        
        m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str(),DOWN_SCALE);


        fsSettings["image_topic"] >> IMAGE_TOPIC;        
        fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
        fsSettings["output_path"] >> VINS_RESULT_PATH;
        fsSettings["save_image"] >> DEBUG_IMAGE;

        // create folder if not exists
        FileSystemHelper::createDirectoryIfNotExists(POSE_GRAPH_SAVE_PATH.c_str());
        FileSystemHelper::createDirectoryIfNotExists(VINS_RESULT_PATH.c_str());

        VISUALIZE_IMU_FORWARD = fsSettings["visualize_imu_forward"];
        LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];
        FAST_RELOCALIZATION = fsSettings["fast_relocalization"];
        VINS_RESULT_PATH = VINS_RESULT_PATH + "/vins_result_loop.csv";
        std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
        fout.close();
        fsSettings.release();


    }

    fsSettings.release();


}
