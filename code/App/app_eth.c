/**
  ******************************************************************************
  * @file    app_eth.c
  * @brief   Ethernet application layer. Hosts the TCP server that will be the
  *          entry point for CAN / RS485 motor control commands.
  ******************************************************************************
  * @note    本文件由手工维护，不属于 STM32CubeMX 生成范围。
  *          唯一被生成代码调用的入口是 app_eth_init()（在 LWIP/App/lwip.c 里）。
  *
  * 当前阶段：把收到的字节**原样回显**，不做任何协议解析。
  *          用于验证 PC -> PHY -> DMA -> LwIP -> netconn -> 返回 的整条链路。
  *          协议定版后，把 app_eth_session() 里的回显替换成解析 + 业务分发。
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_eth.h"
#include "main.h"
#include "cmsis_os.h"
#include "log.h"
#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/ip_addr.h"
#include "lwip/netbuf.h"
#include "lwip/err.h"

/* Private define ------------------------------------------------------------*/
/* TCP port the motor-control server listens on. */
#define APP_ETH_PORT              5000U
/* Server thread stack, in BYTES (CMSIS-RTOS v2 semantics). */
#define APP_ETH_STACK_SIZE        2048U
/* Server thread priority: above the TCP/IP thread (Normal), below EthIf. */
#define APP_ETH_PRIO              osPriorityAboveNormal

/* Private variables ---------------------------------------------------------*/
static osThreadId_t appEthThreadId = NULL;

static const osThreadAttr_t appEthThreadAttr = {
  .name       = "TcpSrv",
  .stack_size = APP_ETH_STACK_SIZE,
  .priority   = (osPriority_t)APP_ETH_PRIO,
};

/* Private function prototypes -----------------------------------------------*/
static void app_eth_thread(void *argument);
static void app_eth_session(struct netconn *conn);

/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Create the TCP server thread.
  * @note   Must be called after MX_LWIP_Init() has started the TCP/IP stack,
  *         otherwise the netconn can not be allocated.
  */
void app_eth_init(void)
{
  appEthThreadId = osThreadNew(app_eth_thread, NULL, &appEthThreadAttr);

  /* Failing here means the FreeRTOS heap is exhausted, which is a build /
     configuration error rather than a run-time condition. */
  if (appEthThreadId == NULL)
  {
    Error_Handler();
  }
}

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  TCP server thread: create the listening socket, then serve one
  *         client at a time.
  * @param  argument: not used
  */
static void app_eth_thread(void *argument)
{
  struct netconn *conn = NULL;
  struct netconn *client = NULL;
  err_t err;

  LWIP_UNUSED_ARG(argument);

  /* Create + bind + listen, retrying while the LwIP pools are not ready yet. */
  for (;;)
  {
    if (conn == NULL)
    {
      conn = netconn_new(NETCONN_TCP);
    }

    if (conn != NULL)
    {
      err = netconn_bind(conn, IP_ADDR_ANY, APP_ETH_PORT);
      if (err == ERR_OK)
      {
        err = netconn_listen(conn);
        if (err == ERR_OK)
        {
          break;
        }
      }

      netconn_delete(conn);
      conn = NULL;
    }

    osDelay(100);
  }

  for (;;)
  {
    if (netconn_accept(conn, &client) != ERR_OK)
    {
      continue;
    }

    app_eth_session(client);

    netconn_close(client);
    netconn_delete(client);
    client = NULL;
  }
}

/**
  * @brief  Serve a single connected client until it disconnects.
  * @param  conn: accepted netconn
  */
static void app_eth_session(struct netconn *conn)
{
  struct netbuf *buf = NULL;
  void *data = NULL;
  u16_t len = 0;

  for (;;)
  {
    if (netconn_recv(conn, &buf) != ERR_OK)
    {
      /* Client closed the connection, or a receive error occurred. */
      log_printf("ETH", "client disconnected");
      return;
    }

    do
    {
      netbuf_data(buf, &data, &len);

      if (len > 0U)
      {
        /* 原样回显：不解析、不判断、不回复固定格式。
           协议定版后这里替换为 proto 解析 + 业务分发。 */
        if (netconn_write(conn, data, len, NETCONN_COPY) != ERR_OK)
        {
          netbuf_delete(buf);
          return;
        }
      }
    } while (netbuf_next(buf) >= 0);

    netbuf_delete(buf);
    buf = NULL;
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
