/*
 * ESP8266 <-> STM32 UART-to-MQTT bridge.
 *
 * The STM32 has no WiFi or MQTT stack of its own -- by design, it only
 * knows how to speak a small custom text protocol over UART. This ESP8266
 * firmware is the *only* thing in the whole system that understands MQTT.
 * Its job is narrow on purpose: read a line of text from the STM32, and
 * turn it into a PUBLISH.
 *
 * Frames received over UART (comma-separated, newline-terminated):
 *   M,V=<voltage>,I=<current>,T=<temperature>\n
 *   E,[SPI=<COMMUNICATION|STORAGE>,]S=<NORMAL|MEASUREMENT|COMMUNICATION>[,<channel>=<IN|OUT>...]\n
 *
 * Published MQTT topics:
 *   project/measurements  - JSON, one message per "M" frame. NOT retained:
 *                            a stale measurement is worse than no measurement,
 *                            and a fresh one is only ever ~1s away.
 *   project/events        - JSON, one message per "E" frame. Retained, so a
 *                            dashboard that connects (or reconnects) late still
 *                            sees the current alarm state immediately, instead
 *                            of a blank panel until the next state change.
 *   project/status        - "online"/"offline", retained, backed by MQTT's
 *                            Last Will and Testament. Lets any subscriber tell
 *                            "the numbers on screen are live" apart from
 *                            "the numbers on screen are whatever was last seen
 *                            before this device disappeared".
 *   project/errors        - best-effort diagnostics for frames that failed to
 *                            parse. Not retained, not queued -- if MQTT happens
 *                            to be down when a bad frame arrives, that specific
 *                            report is simply lost, same as the frame itself.
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <cstring>

#define UART_RX_BUFFER_SIZE 64
#define MQTT_MESSAGE_SIZE 128

const char* ssid = "YOUR_WIFI_SSID";
const char* pass = "YOUR_WIFI_PASSWORD";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

IPAddress server("YOUR_MQTT_IP");  // set this to your own MQTT broker's IP
const int mqtt_port = 1883;

const char* mqtt_client_id = "esp8266-project1";  // must be unique among all clients connected to this broker

// Shared scratch space for building the outgoing JSON payload. This is a
// global, not a local variable inside the functions that use it -- on this
// project that distinction mattered the hard way once already (the very
// first UART echo sketch declared its buffer inside setup() and then tried
// to read it from loop(), where it simply didn't exist). Anything that has
// to survive from one call to the next, or be shared between functions,
// lives up here.
char mqtt_message[MQTT_MESSAGE_SIZE];

char uart_rx_buffer[UART_RX_BUFFER_SIZE];  // accumulates one incoming UART frame, byte by byte, across many loop() passes
uint8_t i = 0;                             // write position in uart_rx_buffer -- also has to be global for the same reason
uint8_t led_state = 0;                     // flips on every complete frame, purely as a visual "yes, I'm still receiving something" heartbeat

// --- Non-blocking WiFi/MQTT reconnect bookkeeping ---
// Both connections used to be re-established with a `while (!connected) { delay(...); }`
// loop. That works, but it means the whole chip -- including the UART read
// further down in loop() -- freezes for as long as the network is down. The
// STM32 keeps sending frames the whole time regardless; without this fix they
// would simply pile up and overflow the hardware UART buffer while WiFi was
// reconnecting. Tracking "when did I last try" with millis() lets loop() come
// back to the UART on every single pass, network or no network.
unsigned long last_wifi_attempt_ms = 0;
const unsigned long wifi_retry_interval_ms = 5000;

unsigned long last_mqtt_attempt_ms = 0;
const unsigned long mqtt_retry_interval_ms = 2000;

// Reports a frame that could not be parsed. Best-effort only: if MQTT isn't
// connected right now, this just gives up rather than trying to remember the
// error for later -- a lost diagnostic message is an acceptable price for
// never blocking on network I/O from inside a parsing function.
void publish_error(const char* reason, const char* raw_frame) {
  if (!mqttClient.connected()) return;
  char err_msg[MQTT_MESSAGE_SIZE];
  snprintf(err_msg, MQTT_MESSAGE_SIZE, "%s: \"%s\"", reason, raw_frame);
  mqttClient.publish("project/errors", err_msg);
}

// Looks at the WiFi state and, if it's down, tries again -- but no more often
// than once every wifi_retry_interval_ms. Calling WiFi.begin() itself doesn't
// block for long, but calling it over and over on every single loop() pass
// while disconnected would still be wasteful and can interfere with a
// connection attempt that's already in progress.
void maintain_wifi() {
  if (WiFi.status() == WL_CONNECTED) return;

  unsigned long now = millis();
  if (now - last_wifi_attempt_ms >= wifi_retry_interval_ms) {
    last_wifi_attempt_ms = now;
    Serial.println("WiFi disconnected, attempting to reconnect...");
    WiFi.begin(ssid, pass);
  }
}

// Same idea as maintain_wifi(), one layer up: only bother trying MQTT once
// WiFi is actually up, and only retry at a sane pace. The Last Will and
// Testament is registered here, at connect() time, not as a separate step --
// the broker needs to know the "offline" message and the topic to publish it
// on *before* anything goes wrong, or it has nothing to fall back to.
void maintain_mqtt() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqttClient.connected()) return;

  unsigned long now = millis();
  if (now - last_mqtt_attempt_ms >= mqtt_retry_interval_ms) {
    last_mqtt_attempt_ms = now;
    Serial.print("Connecting to MQTT broker...");
    // connect(clientId, willTopic, willQos, willRetain, willMessage):
    // if this device ever drops off without a clean disconnect (power loss,
    // crash, WiFi black hole), the BROKER itself publishes "offline" here on
    // our behalf -- we don't have to be alive to say it.
    if (mqttClient.connect(mqtt_client_id, "project/status", 0, true, "offline")) {
      mqttClient.publish("project/status", "online", true);  // retained, so late subscribers see the current state immediately
      Serial.println(" connected.");
    } else {
      Serial.print(" failed, rc=");
      Serial.println(mqttClient.state());
    }
  }
}

// Grabs the next comma-separated token, splits it on '=', and converts the
// value side to a float. Returns false, leaving *out untouched, if there was
// no next token or it had no '=' in it -- returning a plain float couldn't
// tell "parsed 0.0" apart from "found nothing to parse", so the caller needs
// this extra bit of information to know whether it can trust the result.
bool parse_next_float(float* out) {
  char* token = strtok(NULL, ",\n");
  if (token == NULL) return false;

  char* value_str = strchr(token, '=');
  if (value_str == NULL) return false;

  *out = atof(value_str + 1);
  return true;
}

// Parses "M,V=...,I=...,T=..." and republishes it as JSON. This frame always
// has exactly three fields in a fixed order, so there's no need for anything
// fancier than reading them one after another.
void handle_measurement_frame(char* buf) {
  float voltage, current, temp;

  char* token = strtok(buf, ",\n");  // first token is just the "M" tag, nothing to do with it
  if (token == NULL) { publish_error("Bad M frame", buf); return; }

  if (!parse_next_float(&voltage)) { publish_error("Bad M frame (voltage)", buf); return; }
  if (!parse_next_float(&current)) { publish_error("Bad M frame (current)", buf); return; }
  if (!parse_next_float(&temp))    { publish_error("Bad M frame (temp)", buf); return; }

  snprintf(mqtt_message, MQTT_MESSAGE_SIZE,
           "{\"voltage\":%.1f,\"current\":%.1f,\"temperature\":%.1f}",
           voltage, current, temp);

  mqttClient.publish("project/measurements", mqtt_message);
}

// Parses "E,[SPI=...,]S=...[,<channel>=IN|OUT...]" and republishes it as
// JSON. Unlike the measurement frame, this one does NOT have a fixed shape:
// SPI is only present when there's actually a fault, and the number of
// channel fields depends on how many changed. So instead of reading named
// fields one by one, this just walks every comma-separated token it finds
// and re-emits whatever key=value pairs show up -- it works for today's
// fields (SPI, S, V, I, T) without knowing their names in advance, and it
// would keep working unmodified if a future STM32 firmware added more.
void handle_event_frame(char* buf) {
  strcpy(mqtt_message, "{");
  bool first_field = true;

  char* token = strtok(buf, ",\n");  // first token is the "E" tag
  if (token == NULL) { publish_error("Bad E frame", buf); return; }

  token = strtok(NULL, ",\n");
  while (token != NULL) {
    char* value_str = strchr(token, '=');
    if (value_str != NULL) {
      *value_str = '\0';  // cut the token in two, right at the '=', turning one string into a key and a value
      char* key = token;
      char* value = value_str + 1;

      size_t len = strlen(mqtt_message);
      snprintf(mqtt_message + len, MQTT_MESSAGE_SIZE - len,
               "%s\"%s\":\"%s\"", first_field ? "" : ",", key, value);
      first_field = false;
    }
    // a token with no '=' in it shouldn't happen with a well-formed frame,
    // but if it ever does (corrupted transmission etc.), just skip it
    // instead of treating the whole event as unparseable
    token = strtok(NULL, ",\n");
  }

  size_t len = strlen(mqtt_message);
  snprintf(mqtt_message + len, MQTT_MESSAGE_SIZE - len, "}");

  // retained: a dashboard that opens (or reopens) a while after the last
  // alarm change should see the CURRENT state right away, not a blank panel
  // until the next event happens to occur
  mqttClient.publish("project/events", mqtt_message, true);
}

void setup() {
  Serial.begin(9600);
  Serial.swap();  // moves UART0 from GPIO1/3 (wired internally to the USB/CH340 chip) to GPIO15/13,
                   // so the link to the STM32 and the USB link used for flashing never share a wire

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, led_state);

  mqttClient.setServer(server, mqtt_port);

  WiFi.begin(ssid, pass);  // fire-and-forget: this just starts the connection attempt, maintain_wifi() in loop() takes it from here
}

void loop() {
  maintain_wifi();
  maintain_mqtt();
  mqttClient.loop();  // has to run on every single pass -- this is what actually sends MQTT keep-alives and processes incoming data in the background

  if (Serial.available() != 0) {
    char received_byte = (char)Serial.read();

    if (i < UART_RX_BUFFER_SIZE) {
      uart_rx_buffer[i] = received_byte;
      i++;
    } else {
      // filled the whole buffer without ever seeing a newline -- something's
      // wrong with this frame, so drop it and start clean rather than writing
      // past the end of the array
      i = 0;
      return;
    }

    if (received_byte == '\n') {
      if (uart_rx_buffer[0] == 'M') {
        handle_measurement_frame(uart_rx_buffer);
      } else if (uart_rx_buffer[0] == 'E') {
        handle_event_frame(uart_rx_buffer);
      }

      led_state = !led_state;
      digitalWrite(LED_BUILTIN, led_state);

      memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);  // zeroing the whole buffer (not just up to i) guarantees the next
                                                        // frame is always null-terminated, which is what strtok/strchr rely on
      i = 0;
    }
  }
}
