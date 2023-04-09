// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: spherical_coordinates.proto

#ifndef PROTOBUF_spherical_5fcoordinates_2eproto__INCLUDED
#define PROTOBUF_spherical_5fcoordinates_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3000000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3000000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include </home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/util/system.hh>
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_spherical_5fcoordinates_2eproto();
void protobuf_AssignDesc_spherical_5fcoordinates_2eproto();
void protobuf_ShutdownFile_spherical_5fcoordinates_2eproto();

class SphericalCoordinates;

enum SphericalCoordinates_SurfaceModel {
  SphericalCoordinates_SurfaceModel_EARTH_WGS84 = 1
};
GZ_MSGS_VISIBLE bool SphericalCoordinates_SurfaceModel_IsValid(int value);
const SphericalCoordinates_SurfaceModel SphericalCoordinates_SurfaceModel_SurfaceModel_MIN = SphericalCoordinates_SurfaceModel_EARTH_WGS84;
const SphericalCoordinates_SurfaceModel SphericalCoordinates_SurfaceModel_SurfaceModel_MAX = SphericalCoordinates_SurfaceModel_EARTH_WGS84;
const int SphericalCoordinates_SurfaceModel_SurfaceModel_ARRAYSIZE = SphericalCoordinates_SurfaceModel_SurfaceModel_MAX + 1;

GZ_MSGS_VISIBLE const ::google::protobuf::EnumDescriptor* SphericalCoordinates_SurfaceModel_descriptor();
inline const ::std::string& SphericalCoordinates_SurfaceModel_Name(SphericalCoordinates_SurfaceModel value) {
  return ::google::protobuf::internal::NameOfEnum(
    SphericalCoordinates_SurfaceModel_descriptor(), value);
}
inline bool SphericalCoordinates_SurfaceModel_Parse(
    const ::std::string& name, SphericalCoordinates_SurfaceModel* value) {
  return ::google::protobuf::internal::ParseNamedEnum<SphericalCoordinates_SurfaceModel>(
    SphericalCoordinates_SurfaceModel_descriptor(), name, value);
}
// ===================================================================

class GZ_MSGS_VISIBLE SphericalCoordinates : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.SphericalCoordinates) */ {
 public:
  SphericalCoordinates();
  virtual ~SphericalCoordinates();

  SphericalCoordinates(const SphericalCoordinates& from);

  inline SphericalCoordinates& operator=(const SphericalCoordinates& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const SphericalCoordinates& default_instance();

  void Swap(SphericalCoordinates* other);

  // implements Message ----------------------------------------------

  inline SphericalCoordinates* New() const { return New(NULL); }

  SphericalCoordinates* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const SphericalCoordinates& from);
  void MergeFrom(const SphericalCoordinates& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(SphericalCoordinates* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  typedef SphericalCoordinates_SurfaceModel SurfaceModel;
  static const SurfaceModel EARTH_WGS84 =
    SphericalCoordinates_SurfaceModel_EARTH_WGS84;
  static inline bool SurfaceModel_IsValid(int value) {
    return SphericalCoordinates_SurfaceModel_IsValid(value);
  }
  static const SurfaceModel SurfaceModel_MIN =
    SphericalCoordinates_SurfaceModel_SurfaceModel_MIN;
  static const SurfaceModel SurfaceModel_MAX =
    SphericalCoordinates_SurfaceModel_SurfaceModel_MAX;
  static const int SurfaceModel_ARRAYSIZE =
    SphericalCoordinates_SurfaceModel_SurfaceModel_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  SurfaceModel_descriptor() {
    return SphericalCoordinates_SurfaceModel_descriptor();
  }
  static inline const ::std::string& SurfaceModel_Name(SurfaceModel value) {
    return SphericalCoordinates_SurfaceModel_Name(value);
  }
  static inline bool SurfaceModel_Parse(const ::std::string& name,
      SurfaceModel* value) {
    return SphericalCoordinates_SurfaceModel_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // required .gazebo.msgs.SphericalCoordinates.SurfaceModel surface_model = 1;
  bool has_surface_model() const;
  void clear_surface_model();
  static const int kSurfaceModelFieldNumber = 1;
  ::gazebo::msgs::SphericalCoordinates_SurfaceModel surface_model() const;
  void set_surface_model(::gazebo::msgs::SphericalCoordinates_SurfaceModel value);

  // required double latitude_deg = 2;
  bool has_latitude_deg() const;
  void clear_latitude_deg();
  static const int kLatitudeDegFieldNumber = 2;
  double latitude_deg() const;
  void set_latitude_deg(double value);

  // required double longitude_deg = 3;
  bool has_longitude_deg() const;
  void clear_longitude_deg();
  static const int kLongitudeDegFieldNumber = 3;
  double longitude_deg() const;
  void set_longitude_deg(double value);

  // required double elevation = 4;
  bool has_elevation() const;
  void clear_elevation();
  static const int kElevationFieldNumber = 4;
  double elevation() const;
  void set_elevation(double value);

  // required double heading_deg = 5;
  bool has_heading_deg() const;
  void clear_heading_deg();
  static const int kHeadingDegFieldNumber = 5;
  double heading_deg() const;
  void set_heading_deg(double value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.SphericalCoordinates)
 private:
  inline void set_has_surface_model();
  inline void clear_has_surface_model();
  inline void set_has_latitude_deg();
  inline void clear_has_latitude_deg();
  inline void set_has_longitude_deg();
  inline void clear_has_longitude_deg();
  inline void set_has_elevation();
  inline void clear_has_elevation();
  inline void set_has_heading_deg();
  inline void clear_has_heading_deg();

  // helper for ByteSize()
  int RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  double latitude_deg_;
  double longitude_deg_;
  double elevation_;
  double heading_deg_;
  int surface_model_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_spherical_5fcoordinates_2eproto();
  friend void protobuf_AssignDesc_spherical_5fcoordinates_2eproto();
  friend void protobuf_ShutdownFile_spherical_5fcoordinates_2eproto();

  void InitAsDefaultInstance();
  static SphericalCoordinates* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// SphericalCoordinates

// required .gazebo.msgs.SphericalCoordinates.SurfaceModel surface_model = 1;
inline bool SphericalCoordinates::has_surface_model() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void SphericalCoordinates::set_has_surface_model() {
  _has_bits_[0] |= 0x00000001u;
}
inline void SphericalCoordinates::clear_has_surface_model() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void SphericalCoordinates::clear_surface_model() {
  surface_model_ = 1;
  clear_has_surface_model();
}
inline ::gazebo::msgs::SphericalCoordinates_SurfaceModel SphericalCoordinates::surface_model() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.SphericalCoordinates.surface_model)
  return static_cast< ::gazebo::msgs::SphericalCoordinates_SurfaceModel >(surface_model_);
}
inline void SphericalCoordinates::set_surface_model(::gazebo::msgs::SphericalCoordinates_SurfaceModel value) {
  assert(::gazebo::msgs::SphericalCoordinates_SurfaceModel_IsValid(value));
  set_has_surface_model();
  surface_model_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.SphericalCoordinates.surface_model)
}

// required double latitude_deg = 2;
inline bool SphericalCoordinates::has_latitude_deg() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void SphericalCoordinates::set_has_latitude_deg() {
  _has_bits_[0] |= 0x00000002u;
}
inline void SphericalCoordinates::clear_has_latitude_deg() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void SphericalCoordinates::clear_latitude_deg() {
  latitude_deg_ = 0;
  clear_has_latitude_deg();
}
inline double SphericalCoordinates::latitude_deg() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.SphericalCoordinates.latitude_deg)
  return latitude_deg_;
}
inline void SphericalCoordinates::set_latitude_deg(double value) {
  set_has_latitude_deg();
  latitude_deg_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.SphericalCoordinates.latitude_deg)
}

// required double longitude_deg = 3;
inline bool SphericalCoordinates::has_longitude_deg() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void SphericalCoordinates::set_has_longitude_deg() {
  _has_bits_[0] |= 0x00000004u;
}
inline void SphericalCoordinates::clear_has_longitude_deg() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void SphericalCoordinates::clear_longitude_deg() {
  longitude_deg_ = 0;
  clear_has_longitude_deg();
}
inline double SphericalCoordinates::longitude_deg() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.SphericalCoordinates.longitude_deg)
  return longitude_deg_;
}
inline void SphericalCoordinates::set_longitude_deg(double value) {
  set_has_longitude_deg();
  longitude_deg_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.SphericalCoordinates.longitude_deg)
}

// required double elevation = 4;
inline bool SphericalCoordinates::has_elevation() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void SphericalCoordinates::set_has_elevation() {
  _has_bits_[0] |= 0x00000008u;
}
inline void SphericalCoordinates::clear_has_elevation() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void SphericalCoordinates::clear_elevation() {
  elevation_ = 0;
  clear_has_elevation();
}
inline double SphericalCoordinates::elevation() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.SphericalCoordinates.elevation)
  return elevation_;
}
inline void SphericalCoordinates::set_elevation(double value) {
  set_has_elevation();
  elevation_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.SphericalCoordinates.elevation)
}

// required double heading_deg = 5;
inline bool SphericalCoordinates::has_heading_deg() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void SphericalCoordinates::set_has_heading_deg() {
  _has_bits_[0] |= 0x00000010u;
}
inline void SphericalCoordinates::clear_has_heading_deg() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void SphericalCoordinates::clear_heading_deg() {
  heading_deg_ = 0;
  clear_has_heading_deg();
}
inline double SphericalCoordinates::heading_deg() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.SphericalCoordinates.heading_deg)
  return heading_deg_;
}
inline void SphericalCoordinates::set_heading_deg(double value) {
  set_has_heading_deg();
  heading_deg_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.SphericalCoordinates.heading_deg)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

typedef boost::shared_ptr<gazebo::msgs::SphericalCoordinates> SphericalCoordinatesPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

#ifndef SWIG
namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::gazebo::msgs::SphericalCoordinates_SurfaceModel> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::gazebo::msgs::SphericalCoordinates_SurfaceModel>() {
  return ::gazebo::msgs::SphericalCoordinates_SurfaceModel_descriptor();
}

}  // namespace protobuf
}  // namespace google
#endif  // SWIG

typedef const boost::shared_ptr<gazebo::msgs::SphericalCoordinates const> ConstSphericalCoordinatesPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_spherical_5fcoordinates_2eproto__INCLUDED
