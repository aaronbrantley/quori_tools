#ifndef IMAGE_PUBLISHER_
#define IMAGE_PUBLISHER_

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

class ImagePublisher
{
  /*
  *   http://wiki.ros.org/image_transport/Tutorials/PublishingImages
  */
  private:
    // ros
    ros::Publisher rosPublisher;

    // image transport
    image_transport::Publisher publisher;

    // cv
    cv_bridge::CvImage imageReader;

    // sensor_msgs
    sensor_msgs::Image rosImage;

    // std
    std::string publishTopic;

  protected:

  public:
    /*
    *   publish an image from a specified path
    *   https://answers.ros.org/question/99831/publish-file-to-image-topic/?answer=129176#post-id-129176
    */
    void publishImage (int argc, char ** argv, std::string imagePath)
    {
      // create node named imagePublisher
      ros::init (argc, argv, "imagePublisher");
      ros::NodeHandle imageNode;

      // image transport constructor implicit call doesnt work
      image_transport::ImageTransport imageTransport (imageNode);

      // the topic that quori's projector is subscribed to
      publishTopic = "/quori/face/image";

      // load image from specified path. bgr8 encoding is best for quori's projector
      imageReader.image = cv::imread (imagePath, cv::IMREAD_COLOR);
      imageReader.encoding = "bgr8";
      imageReader.toImageMsg (rosImage);

      rosPublisher = imageNode.advertise <sensor_msgs::Image> (publishTopic, 1);
      ros::Rate loopRate (5);

      while (imageNode.ok ())
      {
        // publish image then shutdown publisher
        rosPublisher.publish (rosImage);
        loopRate.sleep ();
      }
    }
};

#endif
