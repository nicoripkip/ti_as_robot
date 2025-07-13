    #include "network.hpp"
    #include <Arduino.h>
    #include <WiFi.h>
    #include <WiFiClient.h>
    #include <time.h>
    #include "config.hpp"
    #include "buffers.hpp"
    #include <esp_wifi.h>
    #include "PubSubClient.h"
    #include "lwjson/lwjson.h"
    #include "slam.hpp"


    // Static globals needed for this file
    WiFiClient      wifi_client;
    PubSubClient    mqtt_client;
    bool            mqtt_connected;
    bool            wifi_connected;


    WebsocketsClient wsClients[MAX_WS_CLIENTS];
    bool wsConnectedFlags[MAX_WS_CLIENTS];
    unsigned long lastReconnectAttempt = 0;
    int wsRetryCount = 0;
    const int wsMaxRetries = 15;
    const unsigned long wsRetryInterval = 1000;


    // Setup json things
    static lwjson_token_t   tokens[128];
    static lwjson_t         _lwjson;


    /**
     * @brief Callback function to capture all mqtt data from the server
     * 
     * @param topic
     * @param payload
     * @param length
     */
    void mqtt_callback(const char* topic, byte* payload, unsigned int length)
    {
        // Init working buffer for the payload
        static char buffer[MQTT_MAX_PACk_SIZE];
        memset(buffer, 0, MQTT_MAX_PACk_SIZE);

        // Copy contents over to make sure funny characters are not present
        memcpy(buffer, payload, length);

        // Check topics to which the payload was written
        if (strcmp(topic, "/hmi2robot") == 0) {
            Serial.println("Message received from hmi!");
        } else if (strcmp(topic, "/scent/to/robot") == 0) {
            if (lwjson_parse(&_lwjson, buffer) == lwjsonOK) {
                const lwjson_token_t* x = lwjson_find(&_lwjson, "x");
                const lwjson_token_t* y = lwjson_find(&_lwjson, "y");
                const lwjson_token_t* scent = lwjson_find(&_lwjson, "scent");

                if (x && y && scent) {
                    float _x = x->u.num_real - 10.0f;
                    float _y = y->u.num_real - 10.0f;
                    float _scent = scent->u.num_real;

                    update_pheromone(_x, _y, _scent);
                }
            }
        } else if (strcmp(topic, "/global") == 0) {
            if (lwjson_parse(&_lwjson, buffer) == lwjsonOK) {
                const lwjson_token_t* bot = lwjson_find(&_lwjson, "name");
                const lwjson_token_t* action = lwjson_find(&_lwjson, "command");

                char botname[10];

                memset(botname, 0, 10);
                memcpy(botname, bot->u.str.token_value, bot->u.str.token_value_len);

                // Safety check
                if (bot && action) {
                    // Check if the message is for this robot
                    if (strcmp(botname, DEVICE_NAME) == 0) {
                        char command[20];

                        memset(command, 0,20);
                        memcpy(command, action->u.str.token_value, action->u.str.token_value_len);
                        
                        BaseType_t err;
                        if (strcmp(command, "idle") == 0) {
                            err = xSemaphoreTake(global_object_state.semaphore, 10);
                            while (err != pdTRUE) err = xSemaphoreTake(global_object_state.semaphore, 10);

                            global_object_state.action = ACTION_IDLE;

                            xSemaphoreGive(global_object_state.semaphore);
                        } else if (strcmp(command, "searching") == 0) {
                            err = xSemaphoreTake(global_object_state.semaphore, 10);
                            while (err != pdTRUE) err = xSemaphoreTake(global_object_state.semaphore, 10);

                            global_object_state.action = ACTION_SEARCHING;

                            xSemaphoreGive(global_object_state.semaphore);
                        }
                    }
                }
            } else {
                Serial.println("Error, parsing gone wrong!");
            }
        }
    }


    /**
     * @brief Function to detect the macaddress of the ESP32
     * 
     */
    void read_mac_address(){
        uint8_t baseMac[6];
        
        // Get MAC address of the WiFi station interface
        esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
        Serial.print("Station MAC: ");
        for (int i = 0; i < 5; i++) {
        Serial.printf("%02X:", baseMac[i]);
        }
        Serial.printf("%02X\n", baseMac[5]);
    }


    /**
     * @brief Function to initialize the wifi network
     * 
     */
    void init_Wifi() 
    {
        read_mac_address();
        wifi_connected = false;

        // Before the loop can start, it is required that a stable network connection is needed!
        // WiFi.setMinSecurity(WIFI_AUTH_WEP); 
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

        // Serial.print("Wifi is trying to connect to the network: ");

        // Try to connect to the network
        uint8_t counter = 0;
        while (WiFi.status() != WL_CONNECTED) {
            Serial.print(".");
            counter++;
            delay(500);

            if (counter >= 100) return;
        }

        // Serial.println("");
        // Serial.println("Wifi connected to network!");

        wifi_connected = true;

        // Turn energy down of the wifi
        // esp_wifi_set_max_tx_power(20);

        // Sync time with the time server
        // Serial.println("Trying to sync time!");
        configTime(0, (3600 * 2), "ntppool1.time.nl", "ntppool2.time.nl");
    }


    /**
     * @brief Function to initialize the mqtt server before it can be used
     * 
     */
    void init_mqtt()
    {
        mqtt_connected = false;

        // Set basic settings of mqtt
        mqtt_client.setClient(wifi_client);
        mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);
        mqtt_client.setCallback(mqtt_callback);

        // Serial.println("Try to connect to mqtt server!");
        if (!MQTT_CLIENT_ANONYMOUS) mqtt_connected = mqtt_client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS);
        else mqtt_connected = mqtt_client.connect(MQTT_CLIENT_ID);

        // if (mqtt_connected) Serial.println("Succesfully connected to mqtt server");
        // else Serial.println("Failed to connect to mqtt server!");

        // Subscribe to the rx channels
        mqtt_client.subscribe("/global");
        mqtt_client.subscribe("/ti/as/hypervisor2robot");

        // Setup MQTT buffers
        mqtt_data_queue = xQueueCreate(10, sizeof(char) * 256);
        logger_queue = xQueueCreate(10, sizeof(char) * MQTT_MAX_PACk_SIZE);
        mqtt_map_queue = xQueueCreate(10, sizeof(char) * MQTT_MAX_PACk_SIZE);

        // init json
        lwjson_init(&_lwjson, tokens, LWJSON_ARRAYSIZE(tokens));
    }


    /**
     * @brief 
     * 
     * @param index
     * @param event
     * @param data
     */
    void onWebSocketEvent(int index, WebsocketsEvent event, String data)
    {
        switch (event)
        {
            case WebsocketsEvent::ConnectionOpened:
                Serial.printf("[EVENT] WebSocket[%d] Connected!\n", index);
                wsConnectedFlags[index] = true;
                break;

            case WebsocketsEvent::ConnectionClosed:
                Serial.printf("[EVENT] WebSocket[%d] Disconnected!\n", index);
                wsConnectedFlags[index] = false;
                break;

            case WebsocketsEvent::GotPing:
                Serial.printf("[EVENT] WebSocket[%d] Got PING\n", index);
                break;

            case WebsocketsEvent::GotPong:
                Serial.printf("[EVENT] WebSocket[%d] Got PONG\n", index);
                break;


            default:
                Serial.printf("[EVENT] WebSocket[%d] Unknown event: %d, data: %s\n", index, (int)event, data.c_str());
                break;
        }
    }


    /**
     * @brief
     * 
     * @param i
     * @param msg
     */
    void onWebSocketMessage(int i, WebsocketsMessage msg)
    {
        if (msg.isText()) {
            String text = msg.data();
            // Serial.printf("WebSocket[%d] Received text: %s\n", i, text.c_str());

            if (text == "DETECTED" && !global_object_state.found_object) {
                // Serial.printf("WebSocket[%d] Detected object!\n", i);

                if (xSemaphoreTake(global_object_state.semaphore, 10) == pdTRUE) {

                    global_object_state.found_object = true;
                    global_object_state.action = ACTION_RETRIEVING;

                    xSemaphoreGive(global_object_state.semaphore);
                }
                // You can trigger GPIO, LED, etc. here
            }
        } else if (msg.isBinary()) {
            Serial.printf("WebSocket[%d] Received binary (%d bytes)\n", i, msg.length());
            // Handle image stream, etc.
        }
    }


    /**
     * @brief
     * 
     */
    void init_websockets()
    {
        for (int i = 0; i < MAX_WS_CLIENTS; ++i)
        {
            // Bind index to callback
            wsClients[i].onEvent([i](WebsocketsEvent event, String data) {
                onWebSocketEvent(i, event, data);
            });

            wsClients[i].onMessage([i](WebsocketsMessage msg) {
                onWebSocketMessage(i, msg);
            });

            String url = String("ws://") + WS_SERVER_HOST + ":" + String(WS_SERVER_PORT) + "/ws";

            // Add custom header only if i > 0
            String clientId = String(DEVICE_NAME);
            if (i > 0) {
                clientId = String(DEVICE_NAME) + String(i);  // e.g., robot_luco1
            }
            wsClients[i].addHeader("X-Client-ID", clientId);

            if (wsClients[i].connect(url)) {
                Serial.printf("WebSocket[%d] connected to: %s\n", i, url.c_str());
            } else {
                Serial.printf("WebSocket[%d] FAILED to connect to: %s\n", i, url.c_str());
            }
        }
    }


    /**
     * @brief Function to connect to the internet and process any network activity
     *
     * @param param
     */
    void network_task(void* param)
    {
        init_Wifi();
        if (wifi_connected) init_mqtt();
        if (wifi_connected) init_websockets();

        while (true)
        {
            if (wifi_connected && mqtt_connected)
            {
                // MQTT message sending
                char message[256];
                memset(message, 0, 256);

                // Send telemetry of the robot onto the mqtt server
                if (uxQueueMessagesWaiting(mqtt_data_queue) > 0) {
                    xQueueReceive(mqtt_data_queue, &message, 5);

                    mqtt_client.publish("/robot", message);
                }

                // Send the map data onto the mqtt server
                if (uxQueueMessagesWaiting(mqtt_map_queue) > 5) { 
                    char map_buff[MQTT_MAX_PACk_SIZE];
                    memset(map_buff, 0, MQTT_MAX_PACk_SIZE);

                    int c = 0;
                    while (xQueueReceive(mqtt_map_queue, &map_buff, 5)) { 
                        mqtt_client.publish(String(String("/map/") + String(DEVICE_NAME)).c_str() , map_buff);

                        c++;

                        if (c == 4) {
                            while (xQueueReceive(mqtt_map_queue, &map_buff, 5));   
                            break;
                        }
                    }
                }

                if (uxQueueMessagesWaiting(scent_map_queue) > 0) {
                    char scent_buff[50];
                    memset(scent_buff, 0, 50);

                    xQueueReceive(scent_map_queue, &scent_buff, 5);

                    mqtt_client.publish("/scent/to/hypervisor", scent_buff);
                }

                mqtt_client.loop();
            }
            else {
                Serial.println("MQTT or WebSocket not connected!");
            }

            for (int i = 0; i < MAX_WS_CLIENTS; ++i)
            {
                wsClients[i].poll();
                if (!wsConnectedFlags[i] && wsRetryCount < wsMaxRetries)
                {
                    unsigned long now = millis();
                    if (now - lastReconnectAttempt >= wsRetryInterval)
                    {
                        String url = String("ws://") + WS_SERVER_HOST + ":" + String(WS_SERVER_PORT) + "/ws?id=" + String(i);
                        Serial.printf("WebSocket[%d] Reconnect attempt %d/%d\n", i, wsRetryCount + 1, wsMaxRetries);

                        if (wsClients[i].connect(url))
                        {
                            Serial.printf("WebSocket[%d] Reconnected!\n", i);
                            wsConnectedFlags[i] = true;
                        }
                        else
                        {
                            Serial.printf("WebSocket[%d] Reconnect failed.\n", i);
                        }
                        lastReconnectAttempt = now;
                        wsRetryCount++;
                    }
                }
            }

            vTaskDelay(50 / portTICK_PERIOD_MS);  // small delay before next request
        }
    }

