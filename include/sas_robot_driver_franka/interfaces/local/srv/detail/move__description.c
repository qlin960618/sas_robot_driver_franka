// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from sas_robot_driver_franka_interfaces:srv/Move.idl
// generated code does not contain a copyright notice

#include "sas_robot_driver_franka_interfaces/srv/detail/move__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_sas_robot_driver_franka_interfaces
const rosidl_type_hash_t *
sas_robot_driver_franka_interfaces__srv__Move__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x76, 0xa3, 0xec, 0xbc, 0x8e, 0x93, 0x8b, 0xda,
      0x53, 0x19, 0xe4, 0x53, 0x99, 0xab, 0xd8, 0x56,
      0xe1, 0xd9, 0x5b, 0x16, 0x89, 0xa1, 0xdd, 0x2c,
      0x90, 0xff, 0x8d, 0x05, 0x04, 0x9f, 0xbc, 0x61,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_sas_robot_driver_franka_interfaces
const rosidl_type_hash_t *
sas_robot_driver_franka_interfaces__srv__Move_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x28, 0xf2, 0x7b, 0x10, 0x36, 0xba, 0x86, 0x14,
      0x9b, 0xf5, 0xe1, 0xa9, 0x4a, 0xe6, 0xeb, 0xe4,
      0x52, 0xec, 0x8c, 0xb2, 0xd0, 0x1c, 0xa7, 0x1a,
      0xda, 0x65, 0x18, 0xe8, 0x7f, 0x77, 0xc1, 0x03,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_sas_robot_driver_franka_interfaces
const rosidl_type_hash_t *
sas_robot_driver_franka_interfaces__srv__Move_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x60, 0x1f, 0x58, 0x97, 0xc3, 0xce, 0xdb, 0xd6,
      0x0d, 0x5a, 0xc9, 0xa7, 0xef, 0xc5, 0xec, 0x80,
      0x07, 0xbb, 0x38, 0x09, 0xad, 0x7e, 0x9a, 0x3f,
      0xda, 0xef, 0x0e, 0x05, 0xf8, 0xcc, 0xea, 0x22,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_sas_robot_driver_franka_interfaces
const rosidl_type_hash_t *
sas_robot_driver_franka_interfaces__srv__Move_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x36, 0x9e, 0x09, 0xe2, 0xc6, 0x51, 0x16, 0x17,
      0xf2, 0x50, 0xce, 0xef, 0x73, 0x77, 0xce, 0xd0,
      0xd7, 0xb6, 0x6d, 0x71, 0xbf, 0xb3, 0x6c, 0xec,
      0xc7, 0xc9, 0x82, 0xbe, 0x68, 0xc6, 0xf8, 0x92,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char sas_robot_driver_franka_interfaces__srv__Move__TYPE_NAME[] = "sas_robot_driver_franka_interfaces/srv/Move";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char sas_robot_driver_franka_interfaces__srv__Move_Event__TYPE_NAME[] = "sas_robot_driver_franka_interfaces/srv/Move_Event";
static char sas_robot_driver_franka_interfaces__srv__Move_Request__TYPE_NAME[] = "sas_robot_driver_franka_interfaces/srv/Move_Request";
static char sas_robot_driver_franka_interfaces__srv__Move_Response__TYPE_NAME[] = "sas_robot_driver_franka_interfaces/srv/Move_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char sas_robot_driver_franka_interfaces__srv__Move__FIELD_NAME__request_message[] = "request_message";
static char sas_robot_driver_franka_interfaces__srv__Move__FIELD_NAME__response_message[] = "response_message";
static char sas_robot_driver_franka_interfaces__srv__Move__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field sas_robot_driver_franka_interfaces__srv__Move__FIELDS[] = {
  {
    {sas_robot_driver_franka_interfaces__srv__Move__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {sas_robot_driver_franka_interfaces__srv__Move_Request__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
  {
    {sas_robot_driver_franka_interfaces__srv__Move__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {sas_robot_driver_franka_interfaces__srv__Move_Response__TYPE_NAME, 52, 52},
    },
    {NULL, 0, 0},
  },
  {
    {sas_robot_driver_franka_interfaces__srv__Move__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {sas_robot_driver_franka_interfaces__srv__Move_Event__TYPE_NAME, 49, 49},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription sas_robot_driver_franka_interfaces__srv__Move__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {sas_robot_driver_franka_interfaces__srv__Move_Event__TYPE_NAME, 49, 49},
    {NULL, 0, 0},
  },
  {
    {sas_robot_driver_franka_interfaces__srv__Move_Request__TYPE_NAME, 51, 51},
    {NULL, 0, 0},
  },
  {
    {sas_robot_driver_franka_interfaces__srv__Move_Response__TYPE_NAME, 52, 52},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
sas_robot_driver_franka_interfaces__srv__Move__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {sas_robot_driver_franka_interfaces__srv__Move__TYPE_NAME, 43, 43},
      {sas_robot_driver_franka_interfaces__srv__Move__FIELDS, 3, 3},
    },
    {sas_robot_driver_franka_interfaces__srv__Move__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = sas_robot_driver_franka_interfaces__srv__Move_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = sas_robot_driver_franka_interfaces__srv__Move_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = sas_robot_driver_franka_interfaces__srv__Move_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char sas_robot_driver_franka_interfaces__srv__Move_Request__FIELD_NAME__width[] = "width";
static char sas_robot_driver_franka_interfaces__srv__Move_Request__FIELD_NAME__speed[] = "speed";

static rosidl_runtime_c__type_description__Field sas_robot_driver_franka_interfaces__srv__Move_Request__FIELDS[] = {
  {
    {sas_robot_driver_franka_interfaces__srv__Move_Request__FIELD_NAME__width, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sas_robot_driver_franka_interfaces__srv__Move_Request__FIELD_NAME__speed, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
sas_robot_driver_franka_interfaces__srv__Move_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {sas_robot_driver_franka_interfaces__srv__Move_Request__TYPE_NAME, 51, 51},
      {sas_robot_driver_franka_interfaces__srv__Move_Request__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char sas_robot_driver_franka_interfaces__srv__Move_Response__FIELD_NAME__success[] = "success";

static rosidl_runtime_c__type_description__Field sas_robot_driver_franka_interfaces__srv__Move_Response__FIELDS[] = {
  {
    {sas_robot_driver_franka_interfaces__srv__Move_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
sas_robot_driver_franka_interfaces__srv__Move_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {sas_robot_driver_franka_interfaces__srv__Move_Response__TYPE_NAME, 52, 52},
      {sas_robot_driver_franka_interfaces__srv__Move_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char sas_robot_driver_franka_interfaces__srv__Move_Event__FIELD_NAME__info[] = "info";
static char sas_robot_driver_franka_interfaces__srv__Move_Event__FIELD_NAME__request[] = "request";
static char sas_robot_driver_franka_interfaces__srv__Move_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field sas_robot_driver_franka_interfaces__srv__Move_Event__FIELDS[] = {
  {
    {sas_robot_driver_franka_interfaces__srv__Move_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {sas_robot_driver_franka_interfaces__srv__Move_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {sas_robot_driver_franka_interfaces__srv__Move_Request__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
  {
    {sas_robot_driver_franka_interfaces__srv__Move_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {sas_robot_driver_franka_interfaces__srv__Move_Response__TYPE_NAME, 52, 52},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription sas_robot_driver_franka_interfaces__srv__Move_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {sas_robot_driver_franka_interfaces__srv__Move_Request__TYPE_NAME, 51, 51},
    {NULL, 0, 0},
  },
  {
    {sas_robot_driver_franka_interfaces__srv__Move_Response__TYPE_NAME, 52, 52},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
sas_robot_driver_franka_interfaces__srv__Move_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {sas_robot_driver_franka_interfaces__srv__Move_Event__TYPE_NAME, 49, 49},
      {sas_robot_driver_franka_interfaces__srv__Move_Event__FIELDS, 3, 3},
    },
    {sas_robot_driver_franka_interfaces__srv__Move_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = sas_robot_driver_franka_interfaces__srv__Move_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = sas_robot_driver_franka_interfaces__srv__Move_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 width\n"
  "float32 speed\n"
  "---\n"
  "bool success";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
sas_robot_driver_franka_interfaces__srv__Move__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {sas_robot_driver_franka_interfaces__srv__Move__TYPE_NAME, 43, 43},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 44, 44},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
sas_robot_driver_franka_interfaces__srv__Move_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {sas_robot_driver_franka_interfaces__srv__Move_Request__TYPE_NAME, 51, 51},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
sas_robot_driver_franka_interfaces__srv__Move_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {sas_robot_driver_franka_interfaces__srv__Move_Response__TYPE_NAME, 52, 52},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
sas_robot_driver_franka_interfaces__srv__Move_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {sas_robot_driver_franka_interfaces__srv__Move_Event__TYPE_NAME, 49, 49},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
sas_robot_driver_franka_interfaces__srv__Move__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *sas_robot_driver_franka_interfaces__srv__Move__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *sas_robot_driver_franka_interfaces__srv__Move_Event__get_individual_type_description_source(NULL);
    sources[3] = *sas_robot_driver_franka_interfaces__srv__Move_Request__get_individual_type_description_source(NULL);
    sources[4] = *sas_robot_driver_franka_interfaces__srv__Move_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
sas_robot_driver_franka_interfaces__srv__Move_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *sas_robot_driver_franka_interfaces__srv__Move_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
sas_robot_driver_franka_interfaces__srv__Move_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *sas_robot_driver_franka_interfaces__srv__Move_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
sas_robot_driver_franka_interfaces__srv__Move_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *sas_robot_driver_franka_interfaces__srv__Move_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *sas_robot_driver_franka_interfaces__srv__Move_Request__get_individual_type_description_source(NULL);
    sources[3] = *sas_robot_driver_franka_interfaces__srv__Move_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
