/**
 * Copyright (C) 2019  Nicolas Jeanmonod, ouilogique.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include <Arduino.h>
#include <WifiSettings.h>
#include <Hexapod_WebServerApp.h>

bool ssidFound;
char apssid[33];

/**
 * onWsEvent
 */
void onWsEvent(AsyncWebSocket *server,
               AsyncWebSocketClient *client,
               AwsEventType type,
               void *arg,
               uint8_t *data,
               size_t len)
{
    if (type == WS_EVT_CONNECT)
    {
        Serial.printf("ws[%s][%u] connect\n",
                      server->url(),
                      client->id());
        client->printf("\"Hello Client %u :)\"",
                       client->id());
        client->ping();
    }
    else if (type == WS_EVT_DISCONNECT)
    {
        Serial.printf("ws[%s][%u] disconnect\n",
                      server->url(),
                      client->id());
    }
    else if (type == WS_EVT_ERROR)
    {
        Serial.printf("ws[%s][%u] error(%u): %s\n",
                      server->url(),
                      client->id(),
                      *((uint16_t *)arg),
                      (char *)data);
    }
    else if (type == WS_EVT_PONG)
    {
        Serial.printf("ws[%s][%u] pong[%u]: %s\n",
                      server->url(),
                      client->id(),
                      len,
                      (len) ? (char *)data : "");
    }
    else if (type == WS_EVT_DATA)
    {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        String msg = "";
        if (info->final && info->index == 0 && info->len == len)
        {
            // The whole message is in a single frame and we got all of it's data.
            Serial.printf("ws[%s][%u] %s-message[%llu]: ",
                          server->url(),
                          client->id(),
                          (info->opcode == WS_TEXT) ? "text" : "binary",
                          info->len);

            if (info->opcode == WS_TEXT)
            {
                for (size_t i = 0; i < info->len; i++)
                {
                    msg += (char)data[i];
                }
            }
            else
            {
                char buff[3];
                for (size_t i = 0; i < info->len; i++)
                {
                    sprintf(buff, "%02x ", (uint8_t)data[i]);
                    msg += buff;
                }
            }
            Serial.printf("%s\n", msg.c_str());

            if (info->opcode == WS_TEXT)
                client->text("\"I got your text message (1)\"");
            else
                client->binary("\"I got your binary message (1)\"");
        }
        else
        {
            // Message is comprised of multiple frames or the frame is split into multiple packets.
            if (info->index == 0)
            {
                if (info->num == 0)
                {
                    Serial.printf("ws[%s][%u] %s-message start\n",
                                  server->url(),
                                  client->id(),
                                  (info->message_opcode == WS_TEXT) ? "text" : "binary");
                }

                Serial.printf("ws[%s][%u] frame[%u] start[%llu]\n",
                              server->url(),
                              client->id(),
                              info->num, info->len);
            }

            Serial.printf("ws[%s][%u] frame[%u] %s[%llu - %llu]: ",
                          server->url(),
                          client->id(),
                          info->num,
                          (info->message_opcode == WS_TEXT) ? "text" : "binary",
                          info->index,
                          info->index + len);

            if (info->opcode == WS_TEXT)
            {
                for (size_t i = 0; i < info->len; i++)
                {
                    msg += (char)data[i];
                }
            }
            else
            {
                char buff[3];
                for (size_t i = 0; i < info->len; i++)
                {
                    sprintf(buff, "%02x ", (uint8_t)data[i]);
                    msg += buff;
                }
            }
            Serial.printf("%s\n", msg.c_str());

            if ((info->index + len) == info->len)
            {
                Serial.printf("ws[%s][%u] frame[%u] end[%llu]\n",
                              server->url(),
                              client->id(),
                              info->num,
                              info->len);
                if (info->final)
                {
                    Serial.printf("ws[%s][%u] %s-message end\n",
                                  server->url(),
                                  client->id(),
                                  (info->message_opcode == WS_TEXT) ? "text" : "binary");

                    if (info->message_opcode == WS_TEXT)
                        client->text("I got your text message (2)");
                    else
                        client->binary("I got your binary message (2)");
                }
            }
        }
    }
}

/**
 *
 */
void writeServerInfoToFile()
{
    File file = SPIFFS.open("/info.json", FILE_WRITE);

    if (!file)
    {
        Serial.println("There was an error opening the file for writing");
        return;
    }

    file.println("{");

    // ws enabled
    file.println(
        R"rawText("ws_enabled":        ")rawText" +
        String(ws.enabled()) +
        R"rawText(",)rawText");

    // ws url
    file.print(
        R"rawText("ws_url":            ")rawText");
    file.print(
        ws.url());
    file.println(
        R"rawText(",)rawText");

    // wifi_local_ip
    file.print(
        R"rawText("wifi_local_ip":     ")rawText");
    file.print(WiFi.localIP());
    file.println(
        R"rawText(",)rawText");

    // wifi_soft_ap_ip
    file.print(
        R"rawText("wifi_soft_ap_ip":   ")rawText");
    file.print(
        WiFi.softAPIP());
    file.println(
        R"rawText(",)rawText");

    // ap_ssid
    file.print(
        R"rawText("ap_ssid":           ")rawText");
    file.print(
        apssid);
    file.println(
        R"rawText(",)rawText");

    // compilation date
    file.print(
        R"rawText("compilation_date":  ")rawText");
    file.print(
        __DATE__);
    file.println(
        R"rawText(",)rawText");

    // compilation time
    file.print(
        R"rawText("compilation_time":  ")rawText");
    file.print(
        __TIME__);
    file.println(
        R"rawText(")rawText");

    file.print("}");

    file.close();
}

/**
 * setupWebServer
 */
void setupWebServer()
{
    strcpy(apssid, ap_ssid);
    strcat(apssid, WiFi.macAddress().c_str());
    Serial.print("## apssid = ");
    Serial.println(apssid);

    if (ssidFound)
    {
        Serial.print("WiFi MODE = STATION + SOFT-AP\n");
        WiFi.mode(WIFI_AP_STA);
        WiFi.softAP(apssid);
        WiFi.begin(ssid, password);
        if (WiFi.waitForConnectResult() != WL_CONNECTED)
        {
            Serial.printf("STA: Failed!\n");
            WiFi.disconnect(false);
            delay(1000);
            WiFi.begin(ssid, password);
        }
    }
    else
    {
        Serial.print("WiFi MODE = SOFT-AP\n");
        WiFi.mode(WIFI_AP);
        Serial.print("ap_password = ");
        Serial.println(ap_password);
        if (strlen(ap_password) == 0)
            WiFi.softAP(apssid);
        else
            WiFi.softAP(apssid, ap_password);
    }

    WiFi.setHostname(ap_ssid);

    Serial.print("####\nws.client.enabled: ");
    Serial.print(ws.enabled());
    Serial.print("\n####\nws: ");
    Serial.print(ws.url());
    Serial.print("\n####\n");

    Serial.print("STATION IP address:\nhttp://");
    Serial.print(WiFi.localIP());
    Serial.print("/\n");
    Serial.print("SOFT-AP IP address:\nhttp://");
    Serial.print(WiFi.softAPIP());
    Serial.print("/\n");

    /**
     *
     */
    ArduinoOTA.onStart([]()
                       { events.send("Update Start", "ota"); });

    /**
     *
     */
    ArduinoOTA.onEnd([]()
                     { events.send("Update End", "ota"); });

    /**
     *
     */
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          {
        char p[32];
        sprintf(p, "Progress: %u%%\n", (progress / (total / 100)));
        events.send(p, "ota"); });

    /**
     *
     */
    ArduinoOTA.onError([](ota_error_t error)
                       {
        if (error == OTA_AUTH_ERROR)
            events.send("Auth Failed", "ota");
        else if (error == OTA_BEGIN_ERROR)
            events.send("Begin Failed", "ota");
        else if (error == OTA_CONNECT_ERROR)
            events.send("Connect Failed", "ota");
        else if (error == OTA_RECEIVE_ERROR)
            events.send("Recieve Failed", "ota");
        else if (error == OTA_END_ERROR)
            events.send("End Failed", "ota"); });

    /**
     *
     */
    ArduinoOTA.setHostname(ap_ssid);

    /**
     *
     */
    ArduinoOTA.begin();

    /**
     *
     */
    MDNS.addService("http", "tcp", 80);

    /**
     *
     */
    SPIFFS.begin();
    writeServerInfoToFile();

    /**
     *
     */
    ws.onEvent(onWsEvent);

    /**
     *
     */
    server.addHandler(&ws);

    /**
     *
     */
    events.onConnect([](AsyncEventSourceClient *client)
                     { client->send("hello!", NULL, millis(), 1000); });

    /**
     *
     */
    server.addHandler(&events);

    /**
     *
     */
    // server.addHandler(new SPIFFSEditor(http_username,http_password));

    /**
     *
     */
    server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/plain", String(ESP.getFreeHeap())); });

    /**
     * Scan available networks
     * First request will return 0 results unless you start scan from somewhere else (loop/setup)
     * Do not request more often than 3-5 seconds
     */
    server.on("/scan", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        String json = "[";
        int n = WiFi.scanComplete();
        if (n == -2)
        {
            WiFi.scanNetworks(true);
        }
        else if (n)
        {
            for (int i = 0; i < n; ++i)
            {
                if (i)
                    json += ",";
                json += "\n{";
                json += "\"rssi\":" + String(WiFi.RSSI(i));
                json += ",\"bssid\":\"" + WiFi.BSSIDstr(i) + "\"";
                json += ",\"secure\":" + String(WiFi.encryptionType(i));
                json += ",\"channel\":" + String(WiFi.channel(i));
                json += ",\"ssid\":\"" + WiFi.SSID(i) + "\"";
                json += "}";
            }
            WiFi.scanDelete();
            if (WiFi.scanComplete() == -2)
            {
                WiFi.scanNetworks(true);
            }
        }
        json += "\n]";
        request->send(200, "application/json", json);
        json = String(); });

    /**
     *
     */
    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

    /**
     *
     */
    server.onNotFound([](AsyncWebServerRequest *request)
                      {
        Serial.printf("NOT_FOUND: ");
        if (request->method() == HTTP_GET)
            Serial.printf("GET");
        else if (request->method() == HTTP_POST)
            Serial.printf("POST");
        else if (request->method() == HTTP_DELETE)
            Serial.printf("DELETE");
        else if (request->method() == HTTP_PUT)
            Serial.printf("PUT");
        else if (request->method() == HTTP_PATCH)
            Serial.printf("PATCH");
        else if (request->method() == HTTP_HEAD)
            Serial.printf("HEAD");
        else if (request->method() == HTTP_OPTIONS)
            Serial.printf("OPTIONS");
        else
            Serial.printf("UNKNOWN");
        Serial.printf(" http://%s%s\n", request->host().c_str(), request->url().c_str());

        if (request->contentLength())
        {
            Serial.printf("_CONTENT_TYPE: %s\n", request->contentType().c_str());
            Serial.printf("_CONTENT_LENGTH: %u\n", request->contentLength());
        }

        int headers = request->headers();
        int i;
        for (i = 0; i < headers; i++)
        {
            AsyncWebHeader *h = request->getHeader(i);
            Serial.printf("_HEADER[%s]: %s\n", h->name().c_str(), h->value().c_str());
        }

        int params = request->params();
        for (i = 0; i < params; i++)
        {
            AsyncWebParameter *p = request->getParam(i);
            if (p->isFile())
            {
                Serial.printf("_FILE[%s]: %s, size: %u\n", p->name().c_str(), p->value().c_str(), p->size());
            }
            else if (p->isPost())
            {
                Serial.printf("_POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
            }
            else
            {
                Serial.printf("_GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
            }
        }

        request->send(404); });

    /**
     *
     */
    server.onFileUpload([](AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final)
                        {
        if (!index)
            Serial.printf("UploadStart: %s\n", filename.c_str());
        Serial.printf("%s", (const char *)data);
        if (final)
            Serial.printf("UploadEnd: %s (%u)\n", filename.c_str(), index + len); });

    /**
     *
     */
    server.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
                         {
        if (!index)
            Serial.printf("BodyStart: %u\n", total);
        Serial.printf("%s", (const char *)data);
        if (index + len == total)
            Serial.printf("BodyEnd: %u\n", total); });

    /**
     *
     */
    server.begin();
}

/**
 * inverseBubbleSortIndexes
 */
void inverseBubbleSortIndexes(int inputArray[], int indexes[], int arraySize)
{
    for (int i = 0; i < arraySize; i++)
        indexes[i] = i;

    for (int i = 0; i < arraySize - 1; i++)
    {
        for (int j = 0; j < arraySize - 1 - i; j++)
        {
            if (inputArray[j] < inputArray[j + 1])
            {
                int temp = inputArray[j + 1];
                inputArray[j + 1] = inputArray[j];
                inputArray[j] = temp;
                int tempI = indexes[j + 1];
                indexes[j + 1] = indexes[j];
                indexes[j] = tempI;
            }
        }
    }
}

/**
 * scanNetwork
 */
void scanNetwork()
{
    Serial.println("# NETWORK SCAN");
    WiFi.disconnect();

    // WiFi.scanNetworks will return the number of networks found
    int nbNetworkFound = WiFi.scanNetworks();
    if (nbNetworkFound == 0)
        Serial.println("\tno networks found");
    else
    {
        Serial.print("\t## ");
        Serial.print(nbNetworkFound);
        Serial.println(" NETWORKS FOUND");

        // Sort SSIDs in function of signal strength.
        int RSSIarray[nbNetworkFound];
        int indexes[nbNetworkFound];
        for (int i = 0; i < nbNetworkFound; i++)
            RSSIarray[i] = WiFi.RSSI(i);
        inverseBubbleSortIndexes(RSSIarray, indexes, nbNetworkFound);

        for (int i = 0; i < nbNetworkFound; i++)
        {
            // Print SSID and RSSI for each network found.
            Serial.print("\t");
            if (i < 9)
                Serial.print(" ");
            Serial.print(i + 1);
            Serial.print(": (");
            Serial.print(WiFi.RSSI(indexes[i]));
            Serial.print(" dB) ");
            Serial.print(WiFi.SSID(indexes[i]));
            if (String(ssid) == (WiFi.SSID(indexes[i])))
            {
                Serial.print(" ** ");
                ssidFound = true;
            }
            Serial.println((WiFi.encryptionType(indexes[i]) == WIFI_AUTH_OPEN) ? " (unprotected)" : "");
            Serial.flush();
        }
    }
    Serial.println("");
}
