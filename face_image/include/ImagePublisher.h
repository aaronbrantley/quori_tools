#ifndef IMAGE_PUBLISHER_
#define IMAGE_PUBLISHER_

#include <dirent.h>

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
    std::string packagePath;
    // list of image names in images/
    std::vector <std::string> expressionList;

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
      packagePath = ros::package::getPath ("face_image");
      std::string imagePath;

      getExpressionList ();

      if (!checkExpression (fileName))
      {
        ROS_ERROR ("invalid expression entered");
        printExpressionList ();
        
        // set expression to error image
        fileName = "error";
      }

      // get the path to image given desired expression
      imagePath = packagePath + "/images/" + fileName + ".jpg";

      ROS_DEBUG_STREAM ("path to image: " + imagePath);

      return imagePath;
    }

    /*
    *   check if expression has an associated jpg
    */
    bool checkExpression (std::string expressionName)
    {
      // for every string in expressionList
      for (int index = 0; index < expressionList.size (); index += 1)
      {
        // if expression has an associated jpg
        if (expressionName == expressionList.at (index))
        {
          return true;
        }
      }

      // if no matching jpg was found
      return false;
    }

    /*
    *   get a list of valid expressions
    *   https://stackoverflow.com/a/46105710
    */
    void getExpressionList ()
    {
      std::string expressionsPath = packagePath + "/images/";
      char * expressionsPathChar = & expressionsPath [0];

      if (auto directory = opendir (expressionsPathChar))
      {
        while (auto entry = readdir (directory))
        {
          std::string currentFileName = entry -> d_name;

          // skip entry if name starts with '.'
          if (currentFileName.at (0) == '.')
          {
            continue;
          }

          for (int index = currentFileName.size () - 1; index > 0; index -= 1)
          {
            if (currentFileName.at (index) == '.')
            {
              currentFileName.erase (index, currentFileName.size () - 1);
            }
          }

          expressionList.push_back (currentFileName);
        }

        closedir (directory);
      }
    }

    /*
    *   print list of valid expressions
    */
    void printExpressionList ()
    {
      std::cout << "valid expressions: ";

      for (int index = 0; index < expressionList.size () - 1; index += 1)
      {
        std::cout << expressionList.at (index) << ", ";
      }

      std::cout << expressionList.at (expressionList.size () - 1) << std::endl;
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

      ros::spinOnce ();
      loopRate.sleep ();

      // quori face application does not like latched publishing
      while (imageNode.ok ())
      {
        ros::spinOnce ();

        // publish image to topic
        rosPublisher.publish (rosImage);

        loopRate.sleep ();

        // shutdown the node since the image has been published
        imageNode.shutdown ();
      }
    }
};

#endif
