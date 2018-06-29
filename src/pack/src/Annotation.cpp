#include"Annotation.h"



uint8_t R, G, B = 0;

uint32_t INTENSITY, ID = 0;





AnnotationClass::AnnotationClass(ros::NodeHandle & nh): m_nh(nh), server("server"){

    cloudm = AffectId(cloud);

    SendCloudMarker(true);

}

AnnotationClass::RosMsgCloud2 AnnotationClass::LoadPcd(const std::string & filename){
    

     sensor_msgs::PointCloud2 cloud2;

     pcl::io::loadPCDFile(filename, cloud2);
	
     return cloud2;

}


AnnotationClass::PointXYZRGBCloud AnnotationClass::ColorCopy()
{
sensor_msgs::PointCloud2 ptcloud2;
PointXYZRGBCloud ColorisedCloud;
PointXYZRGB CloudWithRgb;

ptcloud2 = LoadPcd("cloud.pcd");

pcl::fromROSMsg(ptcloud2,ColorisedCloud);


int height = ptcloud2.height;
int width = ptcloud2.width;
int step = ptcloud2.point_step;


float x, y,z = 0;
uint32_t color =0;



ColorisedCloud.resize(width);



for(int i = 0; i< ColorisedCloud.size();++i)
{
   memcpy(&x, &ptcloud2.data[i * step + ptcloud2.fields[0].offset], sizeof (float));
   memcpy(&y, &ptcloud2.data[i * step + ptcloud2.fields[1].offset], sizeof (float));
   memcpy(&z, &ptcloud2.data[i * step + ptcloud2.fields[2].offset], sizeof (float));
   memcpy(&color, &ptcloud2.data[i * step + ptcloud2.fields[3].offset], sizeof(int));



CloudWithRgb.rgb = color;
uint32_t COLOR = *reinterpret_cast<int*>(&CloudWithRgb.rgb);
uint8_t r = (COLOR >> 16) & 0x0000ff;
uint8_t g = (COLOR >> 8)  & 0x0000ff;
uint8_t b = (COLOR)       & 0x0000ff;

               ColorisedCloud[i].r = r;
               ColorisedCloud[i].g = g;
               ColorisedCloud[i].b = b;


}

return ColorisedCloud;
}






AnnotationClass::VelodyneCloud AnnotationClass::AffectId(Velodyne_Data cloud){


    sensor_msgs::PointCloud2 ptcloud2;
    VelodyneCloud  cloudy;
    ptcloud2 = LoadPcd("cloud.pcd");
    pcl::fromROSMsg(ptcloud2,cloudy);



    int height = ptcloud2.height;
    int width = ptcloud2.width;
    int step = ptcloud2.point_step;


    float x, y,z = 0;
    uint32_t intensity, id = 0;



    cloudy.resize(width);



    for(int i = 0; i< width;++i)
    {
       memcpy(&x, &ptcloud2.data[i * step + ptcloud2.fields[0].offset], sizeof (float));
       memcpy(&y, &ptcloud2.data[i * step + ptcloud2.fields[1].offset], sizeof (float));
       memcpy(&z, &ptcloud2.data[i * step + ptcloud2.fields[2].offset], sizeof (float));
      // memcpy(&color, &ptcloud2.data[i * step + ptcloud2.fields[3].offset], sizeof(int));
       memcpy(&intensity, &ptcloud2.data[i * step + ptcloud2.fields[4].offset], sizeof(int));
       memcpy(&id, &ptcloud2.data[i * step + ptcloud2.fields[5].offset], sizeof(int));



           cloudy[i].id = id;

          cloudy[i].intensity = intensity;

    }





    return cloudy;
}

AnnotationClass::InteractiveMarker AnnotationClass::TransformToMarker(const VelodyneCloud &cloud)
{
  const uint64 cloud_size = cloud.size();
  InteractiveMarker marker;
  marker.header.frame_id ="base_link";
  marker.name = "CLOUD_MARKER_NAME";
  marker.description = "";

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  Marker cloud_marker;
  cloud_marker.type = Marker::POINTS;
  cloud_marker.scale.x = m_point_size_multiplier * m_point_size;
  cloud_marker.scale.y = m_point_size_multiplier * m_point_size;
  cloud_marker.scale.z = m_point_size_multiplier * m_point_size;
  cloud_marker.color.r = 1.0;
  cloud_marker.color.g = 1.0;
  cloud_marker.color.b = 1.0;
  cloud_marker.color.a = 1.0;


  AnnotationClass::PointXYZRGBCloud ColorisedPoint;


  ColorisedPoint = ColorCopy();

  cloud_marker.points.resize(cloud_size);
  cloud_marker.colors.resize(cloud_size);
  for(uint64 i = 0; i < cloud_size; i++)
  {

   const Velodyne_Data & pt = cloud[i];

    cloud_marker.points[i].x = pt.x;
    cloud_marker.points[i].y = pt.y;
    cloud_marker.points[i].z = pt.z;



    cloud_marker.colors[i].r = ColorisedPoint[i].r/255.0;
    cloud_marker.colors[i].g = ColorisedPoint[i].g/255.0;
    cloud_marker.colors[i].b = ColorisedPoint[i].b/255.0;
    cloud_marker.colors[i].a = 1.0;
  }


  visualization_msgs::InteractiveMarkerControl points_control;
  points_control.always_visible = true;
  points_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

  points_control.markers.push_back( cloud_marker );

  // add the control to the interactive marker
  marker.controls.push_back( points_control );

  return marker;
}

AnnotationClass::InteractiveMarker AnnotationClass::SelectCloudObject(const VelodyneCloud & cloud)
{


  InteractiveMarker marker;
  marker.header.frame_id = "base_link";
  marker.name = "";
  marker.description = "";



  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;




  Marker cloud_marker;
  cloud_marker.type = Marker::SPHERE_LIST;
  cloud_marker.scale.x = m_point_size_multiplier * m_point_size*1.5;
  cloud_marker.scale.y = m_point_size_multiplier * m_point_size*1.5;
  cloud_marker.scale.z = m_point_size_multiplier * m_point_size*1.5;
  cloud_marker.color.r = 1.0;
  cloud_marker.color.g = 1.0;
  cloud_marker.color.b = 1.0;
  cloud_marker.color.a = 1.0;
  cloud_marker.points.resize(cloud.size());
  cloud_marker.colors.resize(cloud.size());

  cloudout.clear();
  PointIntensity.clear();


  for(uint64 i = 0; i < cloud.size(); i++)
  {
    const Velodyne_Data & pt = cloud[i];
     if( cloud[i].id == ID )
     {



    cloudout.push_back(cloud[i]);



    cloud_marker.points[i].x = pt.x;
    cloud_marker.points[i].y = pt.y;
    cloud_marker.points[i].z = pt.z;

    cloud_marker.colors[i].r = 1.0;
    cloud_marker.colors[i].g = 1.0;
    cloud_marker.colors[i].b = 1.0;
    cloud_marker.colors[i].a = 1.0;
     }

  }


  SaveMe(cloudout);

  visualization_msgs::InteractiveMarkerControl points_control;
  points_control.always_visible = true;
  points_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  points_control.markers.push_back(cloud_marker);
  marker.controls.push_back(points_control);

  return marker;
}

void AnnotationClass::SendCloudSelection()
{

    server.insert(
    SelectCloudObject(cloudm),boost::bind(&AnnotationClass::GetSelection,this,_1));

    server.applyChanges();

}



void  AnnotationClass::GetSelection(const InteractiveMarkerFeedbackConstPtr  &feedback)
{



    float eps = 0.03;


    uint32_t id = 0;

    uint8_t type = feedback->event_type;

    if( type == InteractiveMarkerFeedback::BUTTON_CLICK )
    {

      if( feedback->mouse_point_valid )
      {


         const uint64 cloud_size = cloudm.size();
         for(uint64 i = 0; i < cloud_size; i++)
         {


            if((fabs(cloudm[i].x-feedback->mouse_point.x))<=eps  && (fabs(cloudm[i].y-feedback->mouse_point.y))<=eps && (fabs(cloudm[i].z-feedback->mouse_point.z))<=eps)
             {



                 id = cloudm[i].id;


                ID= id;



                SendCloudSelection();


              }


         }


    }


    }


}


void AnnotationClass::SendCloudMarker(const bool apply)
{
  server.insert(TransformToMarker(cloudm),boost::bind(&AnnotationClass::GetSelection,this,_1));
  if (apply);
  server.applyChanges();



}


void AnnotationClass::SaveMe(VelodyneCloud cloud2)
{

    std::string name="car" + std::to_string(IndexName) + ".pcd";

    pcl::io::savePCDFileBinary(name, cloud2);



     IndexName++;
}


