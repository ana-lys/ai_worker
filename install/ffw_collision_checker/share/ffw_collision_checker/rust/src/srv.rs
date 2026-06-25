#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to ffw_collision_checker__srv__SolveCollisionNaive_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SolveCollisionNaive_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub joint_names: Vec<std::string::String>,

}



impl Default for SolveCollisionNaive_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SolveCollisionNaive_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SolveCollisionNaive_Request {
  type RmwMsg = super::srv::rmw::SolveCollisionNaive_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        joint_names: msg.joint_names
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        joint_names: msg.joint_names
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      joint_names: msg.joint_names
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
    }
  }
}


// Corresponds to ffw_collision_checker__srv__SolveCollisionNaive_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SolveCollisionNaive_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SolveCollisionNaive_Response {
  type RmwMsg = super::srv::rmw::SolveCollisionNaive_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accept: msg.accept,
        error: msg.error,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accept: msg.accept,
      error: msg.error,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accept: msg.accept,
      error: msg.error,
    }
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


