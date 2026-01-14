// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dss_lio_sam:srv/SaveMap.idl
// generated code does not contain a copyright notice

#ifndef DSS_LIO_SAM__SRV__DETAIL__SAVE_MAP__STRUCT_H_
#define DSS_LIO_SAM__SRV__DETAIL__SAVE_MAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'destination'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SaveMap in the package dss_lio_sam.
typedef struct dss_lio_sam__srv__SaveMap_Request
{
  float resolution;
  rosidl_runtime_c__String destination;
} dss_lio_sam__srv__SaveMap_Request;

// Struct for a sequence of dss_lio_sam__srv__SaveMap_Request.
typedef struct dss_lio_sam__srv__SaveMap_Request__Sequence
{
  dss_lio_sam__srv__SaveMap_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dss_lio_sam__srv__SaveMap_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SaveMap in the package dss_lio_sam.
typedef struct dss_lio_sam__srv__SaveMap_Response
{
  bool success;
} dss_lio_sam__srv__SaveMap_Response;

// Struct for a sequence of dss_lio_sam__srv__SaveMap_Response.
typedef struct dss_lio_sam__srv__SaveMap_Response__Sequence
{
  dss_lio_sam__srv__SaveMap_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dss_lio_sam__srv__SaveMap_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DSS_LIO_SAM__SRV__DETAIL__SAVE_MAP__STRUCT_H_
