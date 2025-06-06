//#include "grbl/hal.h"
//#include "grbl/protocol.h"
// #include "driver.h"

// static user_mcode_ptrs_t user_mcode;

// const user_mcode_t LaserReady_On   = (user_mcode_t)510;
// const user_mcode_t LaserReady_Off  = (user_mcode_t)511;
// const user_mcode_t LaserMains_On   = (user_mcode_t)512;
// const user_mcode_t LaserMains_Off  = (user_mcode_t)513;
// const user_mcode_t LaserGuide_On   = (user_mcode_t)514;
// const user_mcode_t LaserGuide_Off  = (user_mcode_t)515;
// const user_mcode_t LaserEnable_On  = (user_mcode_t)516;
// const user_mcode_t LaserEnable_Off = (user_mcode_t)517;

// const user_mcode_t Argon_On    = (user_mcode_t)520;
// const user_mcode_t Argon_Off   = (user_mcode_t)521;
// const user_mcode_t Powder1_On  = (user_mcode_t)522;
// const user_mcode_t Powder1_Off = (user_mcode_t)523;
// const user_mcode_t Powder2_On  = (user_mcode_t)524;
// const user_mcode_t Powder2_Off = (user_mcode_t)525;

// typedef union {
//     uint8_t bits;                  //!< Bitmask bits
//     uint8_t mask;                  //!< Bitmask
//     uint8_t value;                 //!< Bitmask value
//     struct {
//         uint8_t ready          :1, //!< 
//                 mains          :1, //!< 
//                 guide          :1, //!< 
//                 enable         :1, //!< 
//                 unused         :4;
//     };
// } IPG_state_t;

// typedef union {
//     uint8_t bits;                  //!< Bitmask bits
//     uint8_t mask;                  //!< Bitmask
//     uint8_t value;                 //!< Bitmask value
//     struct {
//         uint8_t argon          :1, //!< 
//                 powder1        :1, //!< 
//                 powder2        :1, //!< 
//                 unused         :5;
//     };
// } BLC_state_t;

// // check - check if M-code is handled here.
// static user_mcode_type_t check (user_mcode_t mcode)
// {
//     return (mcode == LaserReady_On || mcode == LaserReady_Off ||
//             mcode == LaserMains_On || mcode == LaserMains_Off ||
//             mcode == Argon_On || mcode == Argon_Off ||
//             mcode == Powder1_On || mcode == Powder1_Off
//             )
//                      ? UserMCode_Normal //  Handled by us. Set to UserMCode_NoValueWords if there are any parameter words (letters) without an accompanying value.
//                      : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);	// If another handler present then call it or return ignore.
// }

// // validate - validate parameters
// static status_code_t validate (parser_block_t *gc_block)
// {
//     status_code_t state = Status_OK;

//     switch(gc_block->user_mcode) {

//         case LaserReady_On:
//             break;
//         case LaserReady_Off:
//             break;
//         case LaserMains_On:
//             break;
//         case LaserMains_Off:
//             break;
//         case Argon_On:
//             break;
//         case Argon_Off:
//             break;
//         case Powder1_On:
//             break;
//         case Powder1_Off:
//             break;

//         default:
//             state = Status_Unhandled;
//             break;
//     }

//     // If not handled by us and another handler present then call it.
//     return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
// }

// // execute - execute M-code
// static void execute (sys_state_t state, parser_block_t *gc_block)
// {
//     bool handled = true;

//     switch(gc_block->user_mcode) {

//         case LaserReady_On:
//             current_IPG_state.ready = 1;
//             picohal_set_IPG_output(current_IPG_state);
//             break;
//         case LaserReady_Off:
//             current_IPG_state.ready = 0;
//             picohal_set_IPG_output(current_IPG_state);
//             break;
//         case LaserMains_On:
//             current_IPG_state.mains = 1;
//             picohal_set_IPG_output(current_IPG_state);
//             break;
//         case LaserMains_Off:
//             current_IPG_state.mains = 0;
//             picohal_set_IPG_output(current_IPG_state);
//             break;
//         case Argon_On:
//             current_BLC_state.argon = 1;
//             picohal_set_BLC_output(current_BLC_state);
//             break;
//         case Argon_Off:
//             current_BLC_state.argon = 0;
//             picohal_set_BLC_output(current_BLC_state);
//             break;
//         case Powder1_On:
//             current_BLC_state.powder1 = 1;
//             picohal_set_BLC_output(current_BLC_state);
//             break;
//         case Powder1_Off:
//             current_BLC_state.powder1 = 0;
//             picohal_set_BLC_output(current_BLC_state);
//             break;

//         default:
//             handled = false;
//             break;
//     }

//     if(!handled && user_mcode.execute)          // If not handled by us and another handler present
//         user_mcode.execute(state, gc_block);    // then call it.
// }

// // Set up HAL pointers for handling additional M-codes.
// // Call this function on driver setup.
// void mcodes_init (void)
// {
//     // Save away current HAL pointers so that we can use them to keep
//     // any chain of M-code handlers intact.
//     memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

//     // Redirect HAL pointers to our code.
//     grbl.user_mcode.check = check;
//     grbl.user_mcode.validate = validate;
//     grbl.user_mcode.execute = execute;
// }