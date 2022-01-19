#ifndef _ETHERNET_XMC4500_H_
#define _ETHERNET_XMC4500_H_


#include <device.h>
#include <sys/util.h>
#include <kernel.h>
#include <sys/__assert.h>
#include <logging/log.h>
#include <net/net_pkt.h>
#include <net/net_if.h>
#include <net/ethernet.h>
#include <ethernet/eth_stats.h>

#include <xmc_common.h>
#include "xmc_eth_mac.h"
#include "xmc_eth_phy.h"
#include <xmc_gpio.h>

#include "xmc_ccu4.h"
#include "xmc_scu.h"

#include <devicetree.h>

#if defined(CONFIG_PTP_CLOCK_XMC)
#include <drivers/ptp_clock.h>

#define PTP_CLOCK_NAME "PTP_CLOCK"
/* Resolution of sub second counter ~ 20ns */
#define XMC_ETH_SUB_SECOND_UPDATE_FREQ (50000000)
#endif 

/*#define XMC_ETH_PHY_KSZ8081RNB*/

#define ETH_LWIP_0_CRS_DV  XMC_GPIO_PORT15, 9U
#define ETH_LWIP_0_RXER  XMC_GPIO_PORT2, 4U
#define ETH_LWIP_0_RXD0  XMC_GPIO_PORT2, 2U
#define ETH_LWIP_0_RXD1  XMC_GPIO_PORT2, 3U
#define ETH_LWIP_0_RMII_CLK  XMC_GPIO_PORT15, 8U
#define ETH_LWIP_0_MDIO  XMC_GPIO_PORT2, 0U
#define ETH_LWIP_0_TXEN  XMC_GPIO_PORT2, 5U
#define ETH_LWIP_0_TXD0  XMC_GPIO_PORT2, 8U
#define ETH_LWIP_0_TXD1  XMC_GPIO_PORT2, 9U
#define ETH_LWIP_0_MDC  XMC_GPIO_PORT2, 7U
#define ETH_LWIP_0_PIN_LIST_SIZE 10U




#define ETH_LWIP_0_PHY_ADDR   (0)

#define ETH_LWIP_0_NUM_RX_BUF (4U)
#define ETH_LWIP_0_NUM_TX_BUF (4U)

/* MAC ADDRESS*/

/* MAC address*/
#define MAC_ADDR0 (0x00U)
#define MAC_ADDR1 (0x03U)
#define MAC_ADDR2 (0x19U)
#define MAC_ADDR3 (0x45U)
#define MAC_ADDR4 (0x00U)
#define MAC_ADDR5 (0x00U)

#define MAC_ADDR    ((uint64_t)MAC_ADDR0 | \
                     ((uint64_t)MAC_ADDR1 << 8) | \
                     ((uint64_t)MAC_ADDR2 << 16) | \
                     ((uint64_t)MAC_ADDR3 << 24) | \
                     ((uint64_t)MAC_ADDR4 << 32) | \
                     ((uint64_t)MAC_ADDR5 << 40))
#define MAC_ADDR_N(mac)     ((uint64_t)mac[0] | \
                            ((uint64_t)mac[1] << 8) | \
                            ((uint64_t)mac[2] << 16) | \
                            ((uint64_t)mac[3] << 24) | \
                            ((uint64_t)mac[4] << 32) | \
                            ((uint64_t)mac[5] << 40))
/**
 * @brief Enum to indicate runtime error.
 */
typedef enum ETH_LWIP_ERROR
{
  ETH_LWIP_ERROR_NONE,          /**< No error*/
  ETH_LWIP_ERROR_PHY_BUSY,      /**< PHY is busy*/
  ETH_LWIP_ERROR_PHY_ERROR,     /**< PHY status error*/
  ETH_LWIP_ERROR_PHY_DEVICE_ID, /**< Wrong PHY ID used*/
  ETH_LWIP_ERROR_PHY_TIMEOUT    /**< PHY read failed*/
}ETH_LWIP_ERROR_t;

/*Maximum retry iterations for phy auto-negotiation*/
#define ETH_LWIP_PHY_MAX_RETRIES  0xfffffU

/* Rx/Tx DMA descriptors and buffer declarations */
#if defined(__ICCARM__)
#pragma data_alignment=4
static XMC_ETH_MAC_DMA_DESC_t ETH_LWIP_0_rx_desc[ETH_LWIP_0_NUM_RX_BUF];
#pragma data_alignment=4
static XMC_ETH_MAC_DMA_DESC_t ETH_LWIP_0_tx_desc[ETH_LWIP_0_NUM_TX_BUF];
#pragma data_alignment=4
static uint8_t ETH_LWIP_0_rx_buf[ETH_LWIP_0_NUM_RX_BUF][XMC_ETH_MAC_BUF_SIZE];
#pragma data_alignment=4
static uint8_t ETH_LWIP_0_tx_buf[ETH_LWIP_0_NUM_TX_BUF][XMC_ETH_MAC_BUF_SIZE];
#elif defined(__CC_ARM)
static __attribute__((aligned(4))) XMC_ETH_MAC_DMA_DESC_t ETH_LWIP_0_rx_desc[ETH_LWIP_0_NUM_RX_BUF] __attribute__((section ("RW_IRAM1")));
static __attribute__((aligned(4))) XMC_ETH_MAC_DMA_DESC_t ETH_LWIP_0_tx_desc[ETH_LWIP_0_NUM_TX_BUF] __attribute__((section ("RW_IRAM1")));
static __attribute__((aligned(4))) uint8_t ETH_LWIP_0_rx_buf[ETH_LWIP_0_NUM_RX_BUF][XMC_ETH_MAC_BUF_SIZE] __attribute__((section ("RW_IRAM1")));
static __attribute__((aligned(4))) uint8_t ETH_LWIP_0_tx_buf[ETH_LWIP_0_NUM_TX_BUF][XMC_ETH_MAC_BUF_SIZE] __attribute__((section ("RW_IRAM1")));
#elif defined(__GNUC__)
static __attribute__((aligned(4))) XMC_ETH_MAC_DMA_DESC_t ETH_LWIP_0_rx_desc[ETH_LWIP_0_NUM_RX_BUF] __attribute__((section ("ETH_RAM")));
static __attribute__((aligned(4))) XMC_ETH_MAC_DMA_DESC_t ETH_LWIP_0_tx_desc[ETH_LWIP_0_NUM_TX_BUF] __attribute__((section ("ETH_RAM")));
static __attribute__((aligned(4))) uint8_t ETH_LWIP_0_rx_buf[ETH_LWIP_0_NUM_RX_BUF][XMC_ETH_MAC_BUF_SIZE] __attribute__((section ("ETH_RAM")));
static __attribute__((aligned(4))) uint8_t ETH_LWIP_0_tx_buf[ETH_LWIP_0_NUM_TX_BUF][XMC_ETH_MAC_BUF_SIZE] __attribute__((section ("ETH_RAM")));
#else
static __attribute__((aligned(4))) XMC_ETH_MAC_DMA_DESC_t ETH_LWIP_0_rx_desc[ETH_LWIP_0_NUM_RX_BUF];
static __attribute__((aligned(4))) XMC_ETH_MAC_DMA_DESC_t ETH_LWIP_0_tx_desc[ETH_LWIP_0_NUM_TX_BUF];
static __attribute__((aligned(4))) uint8_t ETH_LWIP_0_rx_buf[ETH_LWIP_0_NUM_RX_BUF][XMC_ETH_MAC_BUF_SIZE];
static __attribute__((aligned(4))) uint8_t ETH_LWIP_0_tx_buf[ETH_LWIP_0_NUM_TX_BUF][XMC_ETH_MAC_BUF_SIZE];
#endif

//__nocache

//static __attribute__((aligned(4))) XMC_ETH_MAC_DMA_DESC_t ETH_LWIP_0_rx_desc[ETH_LWIP_0_NUM_RX_BUF] __nocache;
//static __attribute__((aligned(4))) XMC_ETH_MAC_DMA_DESC_t ETH_LWIP_0_tx_desc[ETH_LWIP_0_NUM_TX_BUF] __nocache;
//static __attribute__((aligned(4))) uint8_t ETH_LWIP_0_rx_buf[ETH_LWIP_0_NUM_RX_BUF][XMC_ETH_MAC_BUF_SIZE] __nocache;
//static __attribute__((aligned(4))) uint8_t ETH_LWIP_0_tx_buf[ETH_LWIP_0_NUM_TX_BUF][XMC_ETH_MAC_BUF_SIZE] __nocache;

struct eth_context {
	XMC_ETH_MAC_t eth_mac;
  XMC_ETH_PHY_CONFIG_t eth_phy_config;
	void (*config_func)(void);
	/* If VLAN is enabled, there can be multiple VLAN interfaces related to
	 * this physical device. In that case, this pointer value is not really
	 * used for anything.
	 */
	struct net_if *iface;
	bool enabled;
	bool link_up;
	uint32_t phy_addr;
	uint8_t mac_addr[6];
	void (*generate_mac)(uint8_t *);
#if defined(CONFIG_PTP_CLOCK_XMC)
	const struct device *ptp_clock;
	float clk_ratio;
	float clk_ratio_adj;
#endif
};


#endif //_ETHERNET_XMC4500_H_
