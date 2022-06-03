#ifndef IMAGE_PUBLISHER_
#define IMAGE_PUBLISHER_

#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

/*
*   http://wiki.ros.org/image_transport/Tutorials/PublishingImages
*/
class ImagePublisher
{
  private:
    // ros
    ros::Publisher rosPublisher;

    // cv
    cv_bridge::CvImage imageReader;

    // sensor_msgs
    sensor_msgs::Image rosImage;

    // std
    std::string publishTopic;
    std::string nodeName;

  protected:
    /*
    *   allow calling roscpp functions
    */
    void initializeRos (std::string name)
    {
      int argc = 0;
      char ** argv = nullptr;

      ros::init (argc, argv, name);
    }

    /*
    *   find the image path given the image file name
    */
    std::string getImagePath (std::string fileName)
    {
      // std
      std::string packagePath;
      std::string imagePath;

      // get the path to image given desired expression
      packagePath = ros::package::getPath ("face_image");
      imagePath = packagePath + "/images/" + fileName + ".jpg";

      ROS_DEBUG_STREAM ("path to image: " + imagePath);

      return imagePath;
    }

  public:
    /*
    *   publish an image from a specified path
    *   https://answers.ros.org/question/99831/publish-file-to-image-topic/?answer=129176#post-id-129176
    */
    void publishImage (std::string imageName)
    {
      nodeName = "ImagePublisher";

      initializeRos (nodeName);
      ros::NodeHandle imageNode;
      ros::Rate loopRate (5);

      // image transport constructor implicit call doesnt work
      image_transport::ImageTransport imageTransport (imageNode);

      // the topic that quori's projector is subscribed to
      publishTopic = "/quori/face/image";

      // load image from specified path. bgr8 encoding of a full color 1280x720 image is best for quori's projector
      imageReader.image = cv::imread (getImagePath (imageName), cv::IMREAD_COLOR);
      imageReader.encoding = "bgr8";
      imageReader.toImageMsg (rosImage);

      rosPublisher = imageNode.advertise <sensor_msgs::Image> (publishTopic, 1, true);

      // https://get-help.robotigniteacademy.com/t/how-to-publish-once-only-one-message-into-a-topic-and-get-it-to-work/346
      while (imageNode.ok ())
      {
        // publish image to topic
      	rosPublisher.publish (rosImage);
      	loopRate.sleep ();
      }
    }
};

#endif
