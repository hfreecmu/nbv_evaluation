#include <ros/ros.h>
#include <iostream>
#include <tf2_ros/transform_listener.h>
#include <open3d/Open3D.h>
#include <custom_msgs/GazeboModel.h>
#include <gazebo_msgs/LinkStates.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>


bool readParams(ros::NodeHandle &nhc,
                std::string &worldFrame,
                std::string &robotFrame,
                std::string &resolution,
                std::string &fruitletMeshPath,
                std::string &fruitletMeshName,
                std::string &fruitletLinkIdentifier,
                std::string & outputDir)
{
    if (!nhc.getParam("world_frame", worldFrame))
    {
        ROS_WARN_STREAM("could not read world_frame");
        return false;
    }

    if (!nhc.getParam("robot_frame", robotFrame))
    {
        ROS_WARN_STREAM("could not read robot_frame");
        return false;
    }

    if (!nhc.getParam("resolution", resolution))
    {
        ROS_WARN_STREAM("could not read resolution");
        return false;
    }

    if (!nhc.getParam("fruitlet_mesh_path", fruitletMeshPath))
    {
        ROS_WARN_STREAM("could not read fruitlet_mesh_path");
        return false;
    }

    if (!nhc.getParam("fruitlet_mesh_name", fruitletMeshName))
    {
        ROS_WARN_STREAM("could not read fruitlet_mesh_path");
        return false;
    }

    if (!nhc.getParam("fruitlet_link_identifier", fruitletLinkIdentifier))
    {
        ROS_WARN_STREAM("could not read fruitlet_mesh_path");
        return false;
    }

    if (!nhc.getParam("output_dir", outputDir))
    {
        ROS_WARN_STREAM("could not read output_dir");
        return false;
    }

    return true;
}

bool loadMeshFromModel(std::shared_ptr<open3d::geometry::TriangleMesh> &mesh,
                       std::string &fruitletMeshPath,
                       std::string &targetMeshName
                       ) 
{
    bool success;
    open3d::visualization::rendering::TriangleMeshModel model;
    success = open3d::io::ReadTriangleModel(fruitletMeshPath, model, open3d::io::ReadTriangleModelOptions{false, nullptr});

    if (!success)
    {
        ROS_WARN("not successful reading triangle model");
        return false;
    }

    bool found = false;
    for (open3d::visualization::rendering::TriangleMeshModel::MeshInfo meshInfo : model.meshes_)
    {
        if (meshInfo.mesh_name != targetMeshName)
            continue;

        if (found)
        {
            ROS_WARN_STREAM("More than one mesh found with name: " << targetMeshName);
            return false;
        }

        found = true;

        mesh = meshInfo.mesh;
    } 

    if (!found)
    {
        ROS_WARN_STREAM("Could not find mesh with name: " << targetMeshName);
        return false;
    }

    return true;
}

bool readFruitletPoses(ros::ServiceClient &gazeboParserClient,
                       std::vector<geometry_msgs::Pose> &poses, 
                       std::vector<std::vector<float>> &scales, 
                       std::string &linkIdentifier)
{
    gazebo_msgs::LinkStatesConstPtr linkStates = ros::topic::waitForMessage<gazebo_msgs::LinkStates>("/gazebo/link_states");

    for (int i = 0; i < linkStates->name.size(); i++)
    {
        std::string linkName = linkStates->name[i];
        if (linkName.find(linkIdentifier) == std::string::npos)
            continue;
        
        geometry_msgs::Pose pose = linkStates->pose[i];
        poses.push_back(pose);
        custom_msgs::GazeboModel gazeboParserSrv;
        gazeboParserSrv.request.link_id.data = linkName;

        bool success;
        success = gazeboParserClient.call(gazeboParserSrv);

        if (!success)
        {
            ROS_WARN("Failed to call world parser server");
            return false;
        }
        
        if (!gazeboParserSrv.response.found)
        {
            ROS_WARN("World parser server did not process message correctly");
            return false;
        }

        std::vector<float> scale;
        scale.push_back(gazeboParserSrv.response.scale[0]);
        scale.push_back(gazeboParserSrv.response.scale[1]);
        scale.push_back(gazeboParserSrv.response.scale[2]);

        scales.push_back(scale);
    }

    return true;
}

bool extractGroundTruth(ros::ServiceClient &gazeboParserClient,
                        std::vector<open3d::geometry::TriangleMesh> &singleFruitletMeshes,
                        std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &singleFruitletPointClouds,
                        std::string &fruitletLinkIdentifier,
                        std::shared_ptr<open3d::geometry::TriangleMesh> &fruitletMesh,
                        Eigen::Matrix3d &RWorld,
                        Eigen::Vector3d &tWorld,
                        pcl::PointCloud<pcl::PointXYZRGB> &fullCloud
                        )
{
    std::vector<geometry_msgs::Pose> fruitletPoses;
    std::vector<std::vector<float>> fruitletScales;

    bool success;
    success = readFruitletPoses(gazeboParserClient,
                                fruitletPoses, 
                                fruitletScales, 
                                fruitletLinkIdentifier);

    if (!success)
    {
        ROS_WARN("failed to read fruitlet poses");
        return false;
    }

    for (int i = 0; i < fruitletPoses.size(); i++)
    {
        geometry_msgs::Pose pose = fruitletPoses[i];
        std::vector<float> scale = fruitletScales[i];

        geometry_msgs::Quaternion q = pose.orientation;
        geometry_msgs::Point p = pose.position;
        Eigen::Quaterniond Q(q.w, q.x, q.y, q.z);
        Eigen::Vector3d T(p.x, p.y, p.z);
        Eigen::Matrix3d R = Q.toRotationMatrix();


        //use x scale, all should be same
        //TODO need to fix later maybe
        open3d::geometry::TriangleMesh fruitletMeshInstance(*fruitletMesh);
        fruitletMeshInstance.Scale(scale[0], Eigen::Vector3d(0, 0, 0));
    
        std::vector<Eigen::Vector3d> &meshVertices = fruitletMeshInstance.vertices_;

        pcl::PointCloud<pcl::PointXYZRGB> pointCloud;

        for (Eigen::Vector3d &vertex : meshVertices)
        {
            //TODO probably some way I can do these in all one call
            Eigen::Vector3d vc = Eigen::Vector3d(vertex(0), -vertex(2), vertex(1));
            vc = R * vc + T;
            vc = RWorld * vc + tWorld;

            vertex(0) = vc(0);
            vertex(1) = vc(1);
            vertex(2) = vc(2);

            pcl::PointXYZRGB point;
            point.x = vc(0);
            point.y = vc(1);
            point.z = vc(2);
            point.r = 0;
            point.g = 255;
            point.b = 0;

            pointCloud.push_back(point);
            fullCloud.push_back(point);
        }

        singleFruitletMeshes.push_back(fruitletMeshInstance);
        singleFruitletPointClouds.push_back(pointCloud);
    }

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "extract_gt_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhc("~");

    std::string worldFrame;
    std::string robotFrame;
    std::string resolution;
    std::string fruitletMeshPath;
    std::string fruitletMeshName;
    std::string fruitletLinkIdentifier;
    std::string outputDir;

    bool success;
    success = readParams(nhc, worldFrame, robotFrame, 
                                resolution, fruitletMeshPath, 
                                fruitletMeshName, fruitletLinkIdentifier,
                                outputDir);
    if (!success)
        return 0;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Publisher gtCloudPub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("gt_point_cloud", 1, true);

    //wait a second for tf to become available
    ros::Duration(1.0).sleep();

    geometry_msgs::TransformStamped worldFrameTf;
    try
    {
        worldFrameTf = tfBuffer.lookupTransform(worldFrame, robotFrame, ros::Time::now(), ros::Duration(3.0));
    }
    catch(tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform: %s", ex.what());
        return 0;
    }

    Eigen::Vector3d tWorld(worldFrameTf.transform.translation.x, worldFrameTf.transform.translation.y, worldFrameTf.transform.translation.z);
    Eigen::Quaterniond qWorld(worldFrameTf.transform.rotation.w, worldFrameTf.transform.rotation.x, worldFrameTf.transform.rotation.y, worldFrameTf.transform.rotation.z);
    Eigen::Matrix3d RWorld = qWorld.toRotationMatrix();

    std::shared_ptr<open3d::geometry::TriangleMesh> fruitletMesh;
    success = loadMeshFromModel(fruitletMesh, fruitletMeshPath, fruitletMeshName);

    if (!success)
        return 0;

    ros::ServiceClient gazeboParserClient = nh.serviceClient<custom_msgs::GazeboModel>("world_parser");


    std::vector<open3d::geometry::TriangleMesh> singleFruitletMeshes;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> singleFruitletPointClouds;
    pcl::PointCloud<pcl::PointXYZRGB> fullCloud;

    success = extractGroundTruth(gazeboParserClient,
                                 singleFruitletMeshes,
                                 singleFruitletPointClouds,
                                 fruitletLinkIdentifier,
                                 fruitletMesh,
                                 RWorld,
                                 tWorld,
                                 fullCloud
                                );

    if (!success)
        return 0;

    ROS_INFO("Saving gt fruitlets");
    for (int i = 0; i < singleFruitletMeshes.size(); i++)
    {
        open3d::geometry::TriangleMesh singleFruitetMesh = singleFruitletMeshes[i];
        std::string outputMeshPath = outputDir + "/gt_fruitlet_" + std::to_string(i) + "_mesh.ply";
        open3d::io::WriteTriangleMesh(outputMeshPath, singleFruitetMesh);

        pcl::PointCloud<pcl::PointXYZRGB> singleFruitletPointCloud = singleFruitletPointClouds[i];
        std::string outputPointCloudPath = outputDir + "/gt_fruitlet_" + std::to_string(i) + "_pcd.pcd";
        pcl::io::savePCDFileASCII(outputPointCloudPath, singleFruitletPointCloud);

        //TODO add this back in and also write poses to this file
        // std::string outputDataPath = outputDir + '/gt_fruitlet_' + std::to_string(i) + "_data.txt";
        // std::ofstream f(outputDataPath.c_str(), std::ofstream::out);
        // f << "scale: " << singleFruitletScales[i] << std::endl;
        // f.close();
    }

    std::string outputFullPointCloudPath = outputDir + "/gt_fruitlet_full_pcd.pcd";
    pcl::io::savePCDFileASCII(outputFullPointCloudPath, fullCloud);

    fullCloud.header.frame_id = worldFrame;
    fullCloud.height = 1;
    fullCloud.width = fullCloud.size();
    pcl_conversions::toPCL(ros::Time::now(), fullCloud.header.stamp);
    gtCloudPub.publish(fullCloud);


    ros::waitForShutdown();
}