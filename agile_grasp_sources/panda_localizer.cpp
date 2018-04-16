#include<agile_grasp/panda_localizer.h>

PandaLocalizer::PandaLocalizer(const std::string& svm_filename, const Parameters& params) : pointCloud(new PointCloud())
{
  ROS_INFO("Created Panda grasp class!");
}

void PandaLocalizer::cloud2_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{  
  ROS_INFO("A point cloud has arrived!");
    pcl::fromROSMsg(*msg, *pointCloud);
    ROS_INFO("Transformation done!");
    //std::exit(1);

    localization = new Localization(1,true,false);
     Eigen::VectorXd workspace(6);
    //workspace << 0.4, 0.7, -0.02, 0.06, -0.2, 10;
    //workspace << 0.4, 1.0, -0.3, 0.3, -0.2, 2;
    //workspace << 0.55, 0.9, -0.35, 0.2, -0.2, 2;
    //workspace << 0.6, 0.8, -0.25, 0.1, -0.3, 2;
    // workspace << 0.55, 0.95, -0.25, 0.07, -0.3, 1;
    workspace << -10, 10, -10, 10, -10, 10;
    // workspace << -10, 10, -10, 10, 0.55, 0.95;
  std::vector<double> camera_pose;

    Eigen::Matrix4d R;
  for (int i=0; i < R.rows(); i++)
    R.row(i) << camera_pose[i*R.cols()], camera_pose[i*R.cols() + 1], camera_pose[i*R.cols() + 2], camera_pose[i*R.cols() + 3];  
    
  Eigen::VectorXd ws(6);
  ws << workspace[0], workspace[1], workspace[2], workspace[3], workspace[4], workspace[5];
  workspace = ws;
    localization->setWorkspace(workspace);

    Eigen::Matrix4d base_tf, sqrt_tf;

    base_tf << 0, 0.445417, 0.895323, 0.215, 
               1, 0, 0, -0.015, 
               0, 0.895323, -0.445417, 0.23, 
               0, 0, 0, 1;

    sqrt_tf <<   0.9366,  -0.0162,  0.3500, -0.2863, 
                 0.0151,   0.9999,   0.0058,   0.0058, 
                -0.3501, -0.0002, 0.9367, 0.0554, 
                 0,        0,      0,      1;

    localization->setCameraTransforms(base_tf * sqrt_tf.inverse(), base_tf * sqrt_tf);
    localization->setNumSamples(400);
    localization->setNeighborhoodRadiusTaubin(0.03);
    localization->setNeighborhoodRadiusHands(0.08);
    localization->setFingerWidth(0.01);
    localization->setHandOuterDiameter(0.09);
    localization->setHandDepth(0.06);
    localization->setInitBite(0.01);
    localization->setHandHeight(0.02);
    std::vector<int> indices(0);
    hands=localization->localizeHands(pointCloud,pointCloud->size()/2,indices,true,false);

    antipodal_hands = localization->predictAntipodalHands(hands, "/home/sphero/code/FrankaGrasp/src/agile_grasp/svm_032015_linear_20_20_same");

    std::vector<Handle>handles = localization->findHandles(antipodal_hands, 3, 0.005);

    for(unsigned int i=0;i<antipodal_hands.size();++i)
      antipodal_hands[i].print();
    for(unsigned int i=0;i<hands.size();++i)
      hands[i].print();
    
    

    delete localization;
}

void PandaLocalizer::subscribe(const std::string& cloud_topic, ros::NodeHandle& nh)
{
  this->cloud_topic = nh.subscribe(cloud_topic, 1, &PandaLocalizer::cloud2_callback, this);
}