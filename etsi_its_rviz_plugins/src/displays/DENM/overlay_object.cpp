#include <displays/DENM/overlay_object.hpp>

#include <rclcpp/rclcpp.hpp>
// Using Ogre::PF_A8R8G8B8 in texture creation, so bytes per pixel is 4

namespace rviz_plugin {
ScopedPixelBuffer::ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr pixel_buffer) :
    pixel_buffer_(pixel_buffer) {
  if (pixel_buffer_) {
    pixel_buffer_->lock(Ogre::HardwareBuffer::HBL_NORMAL);
  }
}

ScopedPixelBuffer::~ScopedPixelBuffer() {
  if (pixel_buffer_) {
    pixel_buffer_->unlock();
  }
}

Ogre::HardwarePixelBufferSharedPtr ScopedPixelBuffer::getPixelBuffer() {
  return pixel_buffer_;
}

QImage ScopedPixelBuffer::getQImage(unsigned int width, unsigned int height) {
  if (!pixel_buffer_) {
    return QImage(width, height, QImage::Format_ARGB32);
  }
  const Ogre::PixelBox &pixelBox = pixel_buffer_->getCurrentLock();
  Ogre::uint8 *pDest = static_cast<Ogre::uint8 *> (pixelBox.data);
  const int bytes_per_pixel = 4; // PF_A8R8G8B8
  const int bytes_per_line = static_cast<int>(pixelBox.rowPitch) * bytes_per_pixel;
  // Clear all rows with correct stride
  for (unsigned int row = 0; row < height; ++row) {
    memset(pDest + row * bytes_per_line, 0, bytes_per_line);
  }
  return QImage(pDest, static_cast<int>(width), static_cast<int>(height), bytes_per_line, QImage::Format_ARGB32);
}

QImage ScopedPixelBuffer::getQImage(
    unsigned int width, unsigned int height, QColor &bg_color) {
  QImage Hud = getQImage(width, height);
  if (Hud.isNull()) {
    return Hud;
  }
  Hud.fill(bg_color);
  return Hud;
}

QImage ScopedPixelBuffer::getQImage(OverlayObject &overlay) {
  return getQImage(overlay.getTextureWidth(), overlay.getTextureHeight());
}

QImage ScopedPixelBuffer::getQImage(OverlayObject &overlay,
                                    QColor &bg_color) {
  return getQImage(overlay.getTextureWidth(), overlay.getTextureHeight(),
                   bg_color);
}

OverlayObject::OverlayObject(const std::string &name)
    : name_(name) {
  std::string material_name = name_ + "Material";
  Ogre::OverlayManager *mOverlayMgr = Ogre::OverlayManager::getSingletonPtr();
  overlay_ = mOverlayMgr->create(name_);
  panel_ = static_cast<Ogre::PanelOverlayElement *> (
      mOverlayMgr->createOverlayElement("Panel", name_ + "Panel"));
  panel_->setMetricsMode(Ogre::GMM_PIXELS);

  panel_material_
      = Ogre::MaterialManager::getSingleton().create(
      material_name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  // Ensure technique and pass exist before using
  Ogre::Technique* tech = panel_material_->getTechnique(0);
  if (!tech) { tech = panel_material_->createTechnique(); }
  Ogre::Pass* pass = tech->getPass(0);
  if (!pass) { pass = tech->createPass(); }
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  pass->setDepthWriteEnabled(false);

  panel_->setMaterialName(panel_material_->getName());
  overlay_->add2D(panel_);
  overlay_->setZOrder(500);
}

OverlayObject::~OverlayObject() {
  hide();
  panel_material_->unload();
  Ogre::MaterialManager::getSingleton().remove(panel_material_->getName());
}

std::string OverlayObject::getName() {
  return name_;
}

void OverlayObject::hide() {
  if (overlay_->isVisible()) {
    overlay_->hide();
  }
}

void OverlayObject::show() {
  if (!overlay_->isVisible()) {
    overlay_->show();
  }
}

bool OverlayObject::isTextureReady() {
  return !texture_.isNull();
}

bool OverlayObject::updateTextureSize(unsigned int width, unsigned int height) {
  const std::string texture_name = name_ + "Texture";
  if (width == 0) {
    RCLCPP_WARN(rclcpp::get_logger("OverlayObject"), "[OverlayObject] width=0 is specified as texture size");
    width = 1;
  }
  if (height == 0) {
    RCLCPP_WARN(rclcpp::get_logger("OverlayObject"), "[OverlayObject] height=0 is specified as texture size");
    height = 1;
  }
  if (!isTextureReady() ||
      ((width != texture_->getWidth()) ||
          (height != texture_->getHeight()))) {
    if (isTextureReady()) {
      Ogre::TextureManager::getSingleton().remove(texture_name);
      // Ensure technique/pass exist
      Ogre::Technique* tech = panel_material_->getTechnique(0);
      if (!tech) { tech = panel_material_->createTechnique(); }
      Ogre::Pass* pass = tech->getPass(0);
      if (!pass) { pass = tech->createPass(); }
      pass->removeAllTextureUnitStates();
    }
    texture_ = Ogre::TextureManager::getSingleton().createManual(
        texture_name,        // name
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,   // type
        width, height,   // width & height of the render window 
        0,                   // number of mipmaps
        Ogre::PF_A8R8G8B8,   // pixel format chosen to match a format Qt can use
        Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE     // usage
    );

    Ogre::Technique* tech = panel_material_->getTechnique(0);
    if (!tech) { tech = panel_material_->createTechnique(); }
    Ogre::Pass* pass = tech->getPass(0);
    if (!pass) { pass = tech->createPass(); }
    pass->removeAllTextureUnitStates();
    pass->createTextureUnitState(texture_name);
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(false);
  }
  return isTextureReady();
}

ScopedPixelBuffer OverlayObject::getBuffer() {
  if (isTextureReady()) {
    return ScopedPixelBuffer(texture_->getBuffer());
  } else {
    return ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr());
  }
}

void OverlayObject::setPosition(double left, double top) {
  panel_->setPosition(left, top);
}

void OverlayObject::setDimensions(double width, double height) {
  panel_->setDimensions(width, height);
}

bool OverlayObject::isVisible() {
  return overlay_->isVisible();
}

unsigned int OverlayObject::getTextureWidth() {
  if (isTextureReady()) {
    return texture_->getWidth();
  } else {
    return 0;
  }
}

unsigned int OverlayObject::getTextureHeight() {
  if (isTextureReady()) {
    return texture_->getHeight();
  } else {
    return 0;
  }
}

}  // namespace rviz_plugin
