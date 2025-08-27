#include "displays/DENM/denm_display.hpp"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreBillboardSet.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <QFontDatabase>
#include <QPainter>
#include <QStaticText>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/parse_color.hpp"

#include <rviz_rendering/render_system.hpp>

namespace etsi_its_msgs
{
namespace displays
{

DENMDisplay::DENMDisplay()
{
  // General Properties
  buffer_timeout_ = new rviz_common::properties::FloatProperty(
    "Timeout", 0.1f,
    "Time (in s) until visualizations disappear", this);
  buffer_timeout_->setMin(0);
  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(255, 0, 25),
    "Color", this);
  show_meta_ = new rviz_common::properties::BoolProperty("Metadata", true, 
    "Show metadata as text next to objects", this);
  text_color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(255, 0, 25),
    "Text color", show_meta_);
  char_height_ = new rviz_common::properties::FloatProperty("Scale", 4.0, "Scale of text", show_meta_);
  show_station_id_ = new rviz_common::properties::BoolProperty("StationID", true, 
    "Show StationID", show_meta_);
  show_cause_code_ = new rviz_common::properties::BoolProperty("CauseCode", true, "Show CauseCode", show_meta_);
  show_sub_cause_code_ = new rviz_common::properties::BoolProperty("SubCauseCode", true, "Show SubCauseCode", show_meta_);

  show_overlay_prop_ = new rviz_common::properties::BoolProperty("Overlay", false, "Display DENM as overlay", this);
  bg_color_prop_ = new rviz_common::properties::ColorProperty("BG Color", QColor(255, 85, 0), "Background color of the overlay", show_overlay_prop_);
  bg_alpha_prop_ = new rviz_common::properties::FloatProperty("BG Alpha", 0.2f, "Background alpha of the overlay", bg_color_prop_);
  fg_color_prop_ = new rviz_common::properties::ColorProperty("FG Color", QColor(255, 255, 255), "Foreground color of the overlay", show_overlay_prop_);
  fg_alpha_prop_ = new rviz_common::properties::FloatProperty("FG Alpha", 0.8f, "Foreground alpha of the overlay", fg_color_prop_);
  top_offset_prop_ = new rviz_common::properties::IntProperty("Top Offset", 0, "Vertical offset of the overlay", show_overlay_prop_);
  left_offset_prop_ = new rviz_common::properties::IntProperty("Left Offset", 0, "Horizontal offset of the overlay", show_overlay_prop_);
  width_prop_ = new rviz_common::properties::IntProperty("Width", 128, "Width of the overlay", show_overlay_prop_);
  height_prop_ = new rviz_common::properties::IntProperty("Height", 128, "Height of the overlay", show_overlay_prop_);
  text_size_prop_ = new rviz_common::properties::IntProperty("Text Size", 12, "Font size of the overlay text", show_overlay_prop_);
  line_width_prop_ = new rviz_common::properties::IntProperty("Line Width", 1, "Line width of the overlay", show_overlay_prop_);

  QFontDatabase database;
  font_families_ = database.families();
  font_prop_ = new rviz_common::properties::EnumProperty("Font", "DejaVu Sans", "Font of the overlay text", show_overlay_prop_);
  for (size_t i = 0; i < font_families_.size(); i++) {
    font_prop_->addOption(font_families_[i], (int) i);
  }

  static int i = 0;
  overlay_.reset(new rviz_plugin::OverlayObject("Ogre Overlay number " + std::to_string(i)));
  i++;
}

DENMDisplay::~DENMDisplay()
{
  if (initialized() ) {
    scene_manager_->destroyManualObject(manual_object_);
  }
}

void DENMDisplay::onInitialize()
{
  RTDClass::onInitialize();

  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);

  auto nodeAbstraction = context_->getRosNodeAbstraction().lock();
  rviz_node_ = nodeAbstraction->get_raw_node();

  manual_object_ = scene_manager_->createManualObject();
  manual_object_->setDynamic(true);
  scene_node_->attachObject(manual_object_);
}

void DENMDisplay::reset()
{
  RTDClass::reset();
  manual_object_->clear();
  denms_.clear();
  overlay_->hide();
}

void DENMDisplay::processMessage(etsi_its_denm_msgs::msg::DENM::ConstSharedPtr msg)
{
  // Generate DENM render object from message
  DENMRenderObject denm(*msg);
  if (!denm.validateFloats()) {
        setStatus(
          rviz_common::properties::StatusProperty::Error, "Topic",
          "Message contained invalid floating point values (nans or infs)");
        return;
  }
  
  // Check if Station ID is already present in list
  auto it = denms_.find(denm.getStationID());
  if (it != denms_.end()) it->second = denm; // Key exists, update the value
  else denms_.insert(std::make_pair(denm.getStationID(), denm)); 
  
  overlay_->show();
}

void DENMDisplay::update(float, float) {
    
  // Check for outdated DENMs
  for (auto it = denms_.begin(); it != denms_.end(); ) {
    if (it->second.getAge(rviz_node_->now()) > buffer_timeout_->getFloat()) it = denms_.erase(it);
    else ++it;
  }
  if (denms_.empty() || !show_overlay_prop_->getBool()) {
    overlay_->hide();
  } else {
    updateOverlay(denms_.begin()->second);
  }

  // Render all valid denms
  arrows_.clear();
  texts_.clear();
  for(auto it = denms_.begin(); it != denms_.end(); ++it) {

    DENMRenderObject denm = it->second;
    Ogre::Vector3 sn_position;
    Ogre::Quaternion sn_orientation;
    if (!context_->getFrameManager()->getTransform(denm.getHeader(), sn_position, sn_orientation)) {
      setMissingTransformToFixedFrame(denm.getHeader().frame_id);
      return;
    }
    setTransformOk();

    // set pose of scene node
    scene_node_->setPosition(sn_position);
    scene_node_->setOrientation(sn_orientation);

    auto child_scene_node = scene_node_->createChildSceneNode();
    // Set position of scene node
    geometry_msgs::msg::Pose pose = denm.getPose();
    Ogre::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
    Ogre::Quaternion orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

    //set size parameters for arrow
    int shaft_length = 5;
    int shaft_diameter = 1;
    int head_length = 2;
    int head_diameter = 3;
    position.z += (shaft_length+head_length);
    
    // set pose of child scene node arrow
    child_scene_node->setPosition(position);
    child_scene_node->setOrientation(orientation);
    
    // create arrow object
    std::shared_ptr<rviz_rendering::Arrow> arrow = std::make_shared<rviz_rendering::Arrow>(scene_manager_, child_scene_node, shaft_length, shaft_diameter, head_length, head_diameter);
    
    // set the color of arrow
    Ogre::ColourValue bb_color = rviz_common::properties::qtToOgre(color_property_->getColor());
    arrow->setColor(bb_color);
    arrow->setOrientation(orientation);
    arrows_.push_back(arrow);

    // Visualize meta-information as text
    if(show_meta_->getBool()) {
      std::string text;
      if(show_station_id_->getBool()) {
        text+="StationID: " + std::to_string(denm.getStationID());
        text+="\n";
      }
      if(show_cause_code_->getBool()) {
        text+="Cause: " + denm.getCauseCode();
        text+="\n";
      }
      if(show_sub_cause_code_->getBool()) {
        text+="SubCause: " + denm.getSubCauseCode();
      }
   
      if(!text.size()) return;
      std::shared_ptr<rviz_rendering::MovableText> text_render = std::make_shared<rviz_rendering::MovableText>(text, "Liberation Sans", char_height_->getFloat());
      double height = text_render->getBoundingRadius();
      Ogre::Vector3 offs(0.0, 0.0, height);
      // There is a bug in rviz_rendering::MovableText::setGlobalTranslation https://github.com/ros2/rviz/issues/974
      text_render->setGlobalTranslation(offs);
      Ogre::ColourValue text_color = rviz_common::properties::qtToOgre(text_color_property_->getColor());
      text_render->setColor(text_color);
      child_scene_node->attachObject(text_render.get());
      texts_.push_back(text_render);
    }
  }
}

void DENMDisplay::updateOverlay(DENMRenderObject &denm_render_object) {
  // Prepare overlay text
  std::string general_text = "Hazard warning!";
  std::string subtext;
  subtext += denm_render_object.getCauseCode();
  subtext += "; ";
  subtext += denm_render_object.getSubCauseCode();

  // Prepare font
  QFont overlay_font;
  int font_index = font_prop_->getOptionInt();
  if (font_index < font_families_.size()) {
    overlay_font = QFont(font_families_[font_index]);
  } else {
    RCLCPP_FATAL(rviz_node_->get_logger(), "Unexpected error at selecting font index %d.", font_index);
    return;
  }
  int general_text_size = text_size_prop_->getInt();
  int subtext_size = std::max(1, general_text_size / 2);

  // Measure text sizes
  QFont general_font = overlay_font;
  general_font.setPointSize(general_text_size);
  general_font.setBold(true);

  QFont sub_font = overlay_font;
  sub_font.setPointSize(subtext_size);
  sub_font.setBold(false);

  QFontMetrics general_fm(general_font);
  QFontMetrics sub_fm(sub_font);

  int padding = 12;
  int spacing = 4;

  int general_width = general_fm.horizontalAdvance(QString::fromStdString(general_text));
  int sub_width = sub_fm.horizontalAdvance(QString::fromStdString(subtext));
  int width = std::max(general_width, sub_width) + padding * 2;
  int height = general_fm.height() + sub_fm.height() + spacing + padding * 2;

  // Update overlay texture size to fit text
  overlay_->updateTextureSize(static_cast<unsigned int>(width), static_cast<unsigned int>(height));
  rviz_plugin::ScopedPixelBuffer buffer = overlay_->getBuffer();

  QColor bg_color = bg_color_prop_->getColor();
  bg_color.setAlphaF(bg_alpha_prop_->getFloat());
  QColor fg_color = fg_color_prop_->getColor();
  fg_color.setAlphaF(fg_alpha_prop_->getFloat());

  QImage Hud = buffer.getQImage(*overlay_, bg_color);
  QPainter painter(&Hud);
  painter.setRenderHint(QPainter::Antialiasing, true);

  // Draw rounded rectangle background for a nice look
  painter.setPen(Qt::NoPen);
  painter.setBrush(bg_color);
  painter.drawRoundedRect(0, 0, width, height, 16, 16);

  // Draw general_text (centered)
  painter.setPen(QPen(fg_color, line_width_prop_->getInt() > 0 ? line_width_prop_->getInt() : 1));
  painter.setFont(general_font);
  int general_x = (width - general_width) / 2;
  int general_y = padding + general_fm.ascent();
  painter.drawText(general_x, general_y, QString::fromStdString(general_text));

  // Draw subtext (centered, below general_text)
  painter.setFont(sub_font);
  int sub_x = (width - sub_width) / 2;
  int sub_y = general_y + spacing + sub_fm.ascent();
  painter.drawText(sub_x, sub_y, QString::fromStdString(subtext));

  painter.end();

  overlay_->setDimensions(width, height);
  overlay_->setPosition(left_offset_prop_->getInt(), top_offset_prop_->getInt());
}

}  // namespace displays
}  // namespace etsi_its_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(etsi_its_msgs::displays::DENMDisplay, rviz_common::Display)