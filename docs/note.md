# Node 
## Constructor 1
```cpp
Node::Node(const cv::Mat& visual,
    const cv::Mat& depth,
    const cv::Mat& detection_mask,
    const sensor_msgs::CameraInfoConstPtr& cam_info,
    myHeader depth_header,
    cv::Ptr<cv::Feature2D> detector,
    cv::Ptr<cv::DescriptorExtractor> extractor)

cv::cvtColor(visual, gray_img, CV_RGB2GRAY);
detector->detect(gray_img, feature_locations_2d_, detection_mask); // fill 2d locations
removeDepthless(feature_locations_2d_, depth);
extractor->compute(gray_img, feature_locations_2d_, feature_descriptors_); 
removeDepthless(feature_locations_2d_, depth);
projectTo3D(feature_locations_2d_, feature_locations_3d_, depth, cam_info);
```
- 彩色图像灰度化
- 提取特征点
- 计算描述子
- 将特征点由2D转为3D

## Feature matching
```cpp
unsigned int Node::featureMatching(const Node* other, std::vector<cv::DMatch>* matches) const


## Constructor 2
 
```cpp
Node::Node(const cv::Mat visual,
    cv::Ptr<cv::Feature2D> detector,
    cv::Ptr<cv::DescriptorExtractor> extractor,
    pointcloud_type::Ptr point_cloud,
    const cv::Mat detection_mask)
```
 




