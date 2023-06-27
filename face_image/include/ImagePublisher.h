#ifndef IMAGE_PUBLISHER_H_
#define IMAGE_PUBLISHER_H_

#include <dirent.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

/*
*   generic publisher node creation
*/
class Publisher
{
  protected:
    // ros
    ros::Publisher publisher;

    // standard
    std::string publishTopic;
    std::string nodeName;

    /*
    *   create a ros node given a name
    */
    void initializeRos (std::string name)
    {
      // for init
      int argc = 0;
      char ** argv = nullptr;

      ros::init (argc, argv, name);
    }
};

/*
*   http://wiki.ros.org/image_transport/Tutorials/PublishingImages
*/
class ImagePublisher : public Publisher
{
  private:
    // cv_bridge
    cv_bridge::CvImage imageReader;

    // sensor_msgs
    sensor_msgs::Image rosImage;

    // standard
    std::string packagePath = ros::package::getPath ("face_image");
    std::string imagePath = packagePath + "/images/";
    std::vector <std::string> expressionList;

  protected:
    /*
    *
    */
    void initialize ()
    {
      initializeRos (nodeName);
    }

    /*
    *   find the image path given the image file name
    */
    std::string getImagePath (std::string fileName)
    {
      // if input expression has no file
      if (!checkExpression (fileName))
      {
        ROS_ERROR_STREAM ("invalid expression entered");

        // set expression to error image
        fileName = "error";
      }

      // get the path to image given desired expression
      imagePath += fileName + ".png";

      ROS_DEBUG_STREAM ("path to image: " + imagePath);

      return imagePath;
    }

    /*
    *   get a list of valid expressions
    *   https://stackoverflow.com/a/46105710
    */
    void createExpressionList ()
    {
      expressionList.clear ();

      std::string expressionsPath = packagePath + "/images/";
      // make a character array for dirent functions
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

          // take out the file extension from the expression name
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

  public:
    /*
    *   publish an image from a specified path
    *   if you call this twice, the older publisher will be shutdown
    *   https://answers.ros.org/question/99831/publish-file-to-image-topic/?answer=129176#post-id-129176
    */
    void publishImage (std::string imageName)
    {
      nodeName = "image_publisher";

      initialize ();
      ros::NodeHandle imageNode;
      ros::Rate loopRate (5);

      // image transport constructor implicit call doesnt work
      image_transport::ImageTransport imageTransport (imageNode);

      // the topic that quori's projector is subscribed to
      publishTopic = "/quori/face/image";

      // load image from specified path. bgr8 encoding of a full color 1280x720 image is best for quori's projector
      imageReader.image = cv::imread (getImagePath (imageName), cv::IMREAD_COLOR);
      // might want to change this to rgb8 or rbga8
      imageReader.encoding = "bgr8";
      imageReader.toImageMsg (rosImage);

      publisher = imageNode.advertise <sensor_msgs::Image> (publishTopic, 1);

      // quori face application does not like latched publishing
      while (imageNode.ok ())
      {
        // publish image to topic
        publisher.publish (rosImage);

        loopRate.sleep ();
      }
    }

    /*
    *   print list of valid expressions
    *   gets the name of every file in /images/
    */
    void printExpressionList ()
    {
      createExpressionList ();

      std::cout << "valid expressions: ";

      for (int index = 0; index < expressionList.size () - 1; index += 1)
      {
        std::cout << expressionList.at (index) << ", ";
      }

      std::cout << expressionList.at (expressionList.size () - 1) << std::endl;
    }

    /*
    *   check if expression has an associated png
    */
    bool checkExpression (std::string expressionName)
    {
      createExpressionList ();

      // for every string in expressionList
      for (int index = 0; index < expressionList.size (); index += 1)
      {
        // if expression has an associated png
        if (expressionName == expressionList.at (index))
        {
          return true;
        }
      }

      // if no matching png was found
      return false;
    }
};

#endif
