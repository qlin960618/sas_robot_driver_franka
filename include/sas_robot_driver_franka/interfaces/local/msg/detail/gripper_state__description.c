// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from sas_robot_driver_franka_interfaces:msg/GripperState.idl
// generated code does not contain a copyright notice

#include "sas_robot_driver_franka_interfaces/msg/detail/gripper_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_sas_robot_driver_franka_interfaces
const rosidl_type_hash_t *
sas_robot_driver_franka_interfaces__msg__GripperState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0e, 0x8d, 0x1a, 0xf8, 0x6e, 0x55, 0x6d, 0x06,
      0xb7, 0x37, 0x19, 0xc0, 0x45, 0x07, 0x01, 0xf1,
      0xfb, 0xbb, 0xdd, 0x6e, 0x9c, 0x56, 0x40, 0xa6,
      0xc4, 0xef, 0xf7, 0x34, 0x67, 0xc1, 0xa0, 0x1e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char sas_robot_driver_franka_interfaces__msg__GripperState__TYPE_NAME[] = "sas_robot_driver_franka_interfaces/msg/GripperState";

// Define type names, field names, and default values
static char sas_robot_driver_franka_interfaces__msg__GripperState__FIELD_NAME__width[] = "width";
static char sas_robot_driver_franka_interfaces__msg__GripperState__FIELD_NAME__max_width[] = "max_width";
static char sas_robot_driver_franka_interfaces__msg__GripperState__FIELD_NAME__is_grasped[] = "is_grasped";
static char sas_robot_driver_franka_interfaces__msg__GripperState__FIELD_NAME__temperature[] = "temperature";
static char sas_robot_driver_franka_interfaces__msg__GripperState__FIELD_NAME__duration_ms[] = "duration_ms";

static rosidl_runtime_c__type_description__Field sas_robot_driver_franka_interfaces__msg__GripperState__FIELDS[] = {
  {
    {sas_robot_driver_franka_interfaces__msg__GripperState__FIELD_NAME__width, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sas_robot_driver_franka_interfaces__msg__GripperState__FIELD_NAME__max_width, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sas_robot_driver_franka_interfaces__msg__GripperState__FIELD_NAME__is_grasped, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sas_robot_driver_franka_interfaces__msg__GripperState__FIELD_NAME__temperature, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sas_robot_driver_franka_interfaces__msg__GripperState__FIELD_NAME__duration_ms, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
sas_robot_driver_franka_interfaces__msg__GripperState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {sas_robot_driver_franka_interfaces__msg__GripperState__TYPE_NAME, 51, 51},
      {sas_robot_driver_franka_interfaces__msg__GripperState__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 width\n"
  "float32 max_width\n"
  "bool is_grasped\n"
  "uint16 temperature\n"
  "uint64 duration_ms";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
sas_robot_driver_franka_interfaces__msg__GripperState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {sas_robot_driver_franka_interfaces__msg__GripperState__TYPE_NAME, 51, 51},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 85, 85},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
sas_robot_driver_franka_interfaces__msg__GripperState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *sas_robot_driver_franka_interfaces__msg__GripperState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
