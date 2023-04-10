// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: wireless_nodes.proto

#ifndef PROTOBUF_wireless_5fnodes_2eproto__INCLUDED
#define PROTOBUF_wireless_5fnodes_2eproto__INCLUDED

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
#include <google/protobuf/unknown_field_set.h>
#include "wireless_node.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include </home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/util/system.hh>
#include "/home/ubuntu/wtr_upmachine_ws/src/assembly/include/gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_wireless_5fnodes_2eproto();
void protobuf_AssignDesc_wireless_5fnodes_2eproto();
void protobuf_ShutdownFile_wireless_5fnodes_2eproto();

class WirelessNodes;

// ===================================================================

class GZ_MSGS_VISIBLE WirelessNodes : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.WirelessNodes) */ {
 public:
  WirelessNodes();
  virtual ~WirelessNodes();

  WirelessNodes(const WirelessNodes& from);

  inline WirelessNodes& operator=(const WirelessNodes& from) {
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
  static const WirelessNodes& default_instance();

  void Swap(WirelessNodes* other);

  // implements Message ----------------------------------------------

  inline WirelessNodes* New() const { return New(NULL); }

  WirelessNodes* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const WirelessNodes& from);
  void MergeFrom(const WirelessNodes& from);
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
  void InternalSwap(WirelessNodes* other);
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

  // accessors -------------------------------------------------------

  // repeated .gazebo.msgs.WirelessNode node = 1;
  int node_size() const;
  void clear_node();
  static const int kNodeFieldNumber = 1;
  const ::gazebo::msgs::WirelessNode& node(int index) const;
  ::gazebo::msgs::WirelessNode* mutable_node(int index);
  ::gazebo::msgs::WirelessNode* add_node();
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::WirelessNode >*
      mutable_node();
  const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::WirelessNode >&
      node() const;

  // @@protoc_insertion_point(class_scope:gazebo.msgs.WirelessNodes)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::WirelessNode > node_;
  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_wireless_5fnodes_2eproto();
  friend void protobuf_AssignDesc_wireless_5fnodes_2eproto();
  friend void protobuf_ShutdownFile_wireless_5fnodes_2eproto();

  void InitAsDefaultInstance();
  static WirelessNodes* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// WirelessNodes

// repeated .gazebo.msgs.WirelessNode node = 1;
inline int WirelessNodes::node_size() const {
  return node_.size();
}
inline void WirelessNodes::clear_node() {
  node_.Clear();
}
inline const ::gazebo::msgs::WirelessNode& WirelessNodes::node(int index) const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.WirelessNodes.node)
  return node_.Get(index);
}
inline ::gazebo::msgs::WirelessNode* WirelessNodes::mutable_node(int index) {
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.WirelessNodes.node)
  return node_.Mutable(index);
}
inline ::gazebo::msgs::WirelessNode* WirelessNodes::add_node() {
  // @@protoc_insertion_point(field_add:gazebo.msgs.WirelessNodes.node)
  return node_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::WirelessNode >*
WirelessNodes::mutable_node() {
  // @@protoc_insertion_point(field_mutable_list:gazebo.msgs.WirelessNodes.node)
  return &node_;
}
inline const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::WirelessNode >&
WirelessNodes::node() const {
  // @@protoc_insertion_point(field_list:gazebo.msgs.WirelessNodes.node)
  return node_;
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

typedef boost::shared_ptr<gazebo::msgs::WirelessNodes> WirelessNodesPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

typedef const boost::shared_ptr<gazebo::msgs::WirelessNodes const> ConstWirelessNodesPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_wireless_5fnodes_2eproto__INCLUDED