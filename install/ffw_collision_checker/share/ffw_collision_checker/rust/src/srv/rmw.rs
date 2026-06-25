#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "ffw_collision_checker__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ffw_collision_checker__srv__SolveCollisionNaive_Request() -> *const std::ffi::c_void;
}

#[link(name = "ffw_collision_checker__rosidl_generator_c")]
extern "C" {
    fn ffw_collision_checker__srv__SolveCollisionNaive_Request__init(msg: *mut SolveCollisionNaive_Request) -> bool;
    fn ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SolveCollisionNaive_Request>, size: usize) -> bool;
    fn ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SolveCollisionNaive_Request>);
    fn ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SolveCollisionNaive_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SolveCollisionNaive_Request>) -> bool;
}

// Corresponds to ffw_collision_checker__srv__SolveCollisionNaive_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SolveCollisionNaive_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub joint_names: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,

}



impl Default for SolveCollisionNaive_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ffw_collision_checker__srv__SolveCollisionNaive_Request__init(&mut msg as *mut _) {
        panic!("Call to ffw_collision_checker__srv__SolveCollisionNaive_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SolveCollisionNaive_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ffw_collision_checker__srv__SolveCollisionNaive_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SolveCollisionNaive_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SolveCollisionNaive_Request where Self: Sized {
  const TYPE_NAME: &'static str = "ffw_collision_checker/srv/SolveCollisionNaive_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ffw_collision_checker__srv__SolveCollisionNaive_Request() }
  }
}


#[link(name = "ffw_collision_checker__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ffw_collision_checker__srv__SolveCollisionNaive_Response() -> *const std::ffi::c_void;
}

#[link(name = "ffw_collision_checker__rosidl_generator_c")]
extern "C" {
    fn ffw_collision_checker__srv__SolveCollisionNaive_Response__init(msg: *mut SolveCollisionNaive_Response) -> bool;
    fn ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SolveCollisionNaive_Response>, size: usize) -> bool;
    fn ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SolveCollisionNaive_Response>);
    fn ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SolveCollisionNaive_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SolveCollisionNaive_Response>) -> bool;
}

// Corresponds to ffw_collision_checker__srv__SolveCollisionNaive_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SolveCollisionNaive_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accept: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub error: i32,

}



impl Default for SolveCollisionNaive_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ffw_collision_checker__srv__SolveCollisionNaive_Response__init(&mut msg as *mut _) {
        panic!("Call to ffw_collision_checker__srv__SolveCollisionNaive_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SolveCollisionNaive_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ffw_collision_checker__srv__SolveCollisionNaive_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SolveCollisionNaive_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SolveCollisionNaive_Response where Self: Sized {
  const TYPE_NAME: &'static str = "ffw_collision_checker/srv/SolveCollisionNaive_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ffw_collision_checker__srv__SolveCollisionNaive_Response() }
  }
}






#[link(name = "ffw_collision_checker__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__ffw_collision_checker__srv__SolveCollisionNaive() -> *const std::ffi::c_void;
}

// Corresponds to ffw_collision_checker__srv__SolveCollisionNaive
#[allow(missing_docs, non_camel_case_types)]
pub struct SolveCollisionNaive;

impl rosidl_runtime_rs::Service for SolveCollisionNaive {
    type Request = SolveCollisionNaive_Request;
    type Response = SolveCollisionNaive_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__ffw_collision_checker__srv__SolveCollisionNaive() }
    }
}


