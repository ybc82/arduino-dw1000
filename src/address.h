#ifndef ADDRESS_H__
#define ADDRESS_H__

#if defined(NODE_BASE)
  #define SHORT_ADDRESS {0x00, 0x00}
  #define PIN_NUM_IRQ 2
  #define UNIQUE_ID "82:17:5B:D5:A9:9A:E2:90"
  #define MAX_DEVICES_IN_MEMORY 5
#elif defined(NODE_ANCHOR1)
  #define SHORT_ADDRESS {0x00, 0x01}
  #define PIN_NUM_IRQ 3
  #define UNIQUE_ID "82:17:5B:D5:A9:9A:E2:9E"
  #define MAX_DEVICES_IN_MEMORY 4
#elif defined(NODE_ANCHOR2)
  #define SHORT_ADDRESS {0x00, 0x02}
  #define PIN_NUM_IRQ 2
  #define UNIQUE_ID "82:17:5B:D5:A9:9A:E2:9F"
  #define MAX_DEVICES_IN_MEMORY 4
#elif defined(NODE_ANCHOR3)
  #define SHORT_ADDRESS {0x00, 0x03}
  #define PIN_NUM_IRQ 2
  #define UNIQUE_ID "82:17:5B:D5:A9:9A:E2:9D"
  #define MAX_DEVICES_IN_MEMORY 4
#elif defined(NODE_ANCHOR4)
  #define SHORT_ADDRESS {0x00, 0x04}
  #define PIN_NUM_IRQ 2
  #define UNIQUE_ID "82:17:5B:D5:A9:9A:E2:91"
  #define MAX_DEVICES_IN_MEMORY 4
#elif defined(NODE_ANCHOR5)
  #define SHORT_ADDRESS {0x00, 0x05}
  #define PIN_NUM_IRQ 2
  #define UNIQUE_ID "82:17:5B:D5:A9:9A:E2:92"
  #define MAX_DEVICES_IN_MEMORY 4
#elif defined(NODE_TAG)   // this is the tag in TOF mode; the following are tags in TDOA mode
  #define SHORT_ADDRESS {0x01, 0x01}
  #define PIN_NUM_IRQ 2
  #define UNIQUE_ID "7D:00:22:EA:82:60:3B:9C"
  #define MAX_DEVICES_IN_MEMORY 4
#elif defined(NODE_TAG1)
  #define SHORT_ADDRESS {0x01, 0x01}
  #define PIN_NUM_IRQ 2
  #define UNIQUE_ID "82:17:5B:D5:A9:9A:E2:80"
  #define MAX_DEVICES_IN_MEMORY 6
#elif defined(NODE_TAG2)
  #define SEND_TO_RECV
  #define SHORT_ADDRESS {0x01, 0x02}
  #define PIN_NUM_IRQ 2
  #define UNIQUE_ID "82:17:5B:D5:A9:9A:E2:81"
  #define MAX_DEVICES_IN_MEMORY 6
#elif defined(NODE_RECV)
  #define SHORT_ADDRESS {0x02, 0x01}
  #define PIN_NUM_IRQ 2
  #define UNIQUE_ID "83:17:5B:D5:A9:9A:E2:81"
  #define MAX_DEVICES_IN_MEMORY 6
#else
  #error "Node type unspecified!"
#endif // NODE

#define PIN_NUM_RST 9

#define BASE_ADDRESS        {0x82, 0x17, 0x5B, 0xD5, 0xA9, 0x9A, 0xE2, 0x90}
#define BASE_SHORT_ADDRESS  {0x00, 0x00}
#define ANCHOR1_SHORT_ADDRESS   {0x00, 0x01}
#define ANCHOR2_SHORT_ADDRESS   {0x00, 0x02}
#define ANCHOR3_SHORT_ADDRESS   {0x00, 0x03}
#define ANCHOR4_SHORT_ADDRESS   {0x00, 0x04}
#define ANCHOR5_SHORT_ADDRESS   {0x00, 0x05}

#define TAG_UNIQUE_ID   "7D:00:22:EA:82:60:3B:9C"
#endif // ADDRESS_H__
