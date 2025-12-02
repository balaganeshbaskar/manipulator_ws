// ============================================================
// BASIC ROS2 CONNECTIVITY TEST
// Tests bidirectional serial communication between WSL and Teensy
// ============================================================

unsigned long lastFeedback = 0;
bool initialized = false;

// Simulated joint positions (in radians)
float joint_positions[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    
    pinMode(LED_BUILTIN, OUTPUT);
    
    Serial.println("\n========================================");
    Serial.println("  TEENSY 4.1 - ROS2 CONNECTIVITY TEST");
    Serial.println("========================================");
    Serial.println("Ready. Send <STATUS> or <J1,J2,J3,J4,J5>");
    Serial.println("========================================\n");
}

void loop() {
    // Blink LED (heartbeat)
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 500) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        lastBlink = millis();
    }
    
    // Process commands
    processCommands();
    
    // Send feedback at 20Hz when initialized
    if (initialized && (millis() - lastFeedback >= 50)) {
        sendPositionFeedback();
        lastFeedback = millis();
    }
}

void processCommands() {
    if (!Serial.available()) return;
    
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;
    
    // ROS2-style commands (enclosed in < >)
    if (cmd.startsWith("<") && cmd.endsWith(">")) {
        String content = cmd.substring(1, cmd.length() - 1);
        
        if (content == "STATUS") {
            Serial.println("[Teensy] STATUS request received");
            initialized = true;
            sendPositionFeedback();
        } else {
            parseJointCommand(content);
        }
    } else {
        Serial.print("[Teensy] Unknown: ");
        Serial.println(cmd);
    }
}

void parseJointCommand(String content) {
    Serial.print("[Teensy] Command: <");
    Serial.print(content);
    Serial.println(">");
    
    int jointIdx = 0;
    int startIdx = 0;
    
    for (int i = 0; i <= content.length(); i++) {
        if (i == content.length() || content.charAt(i) == ',') {
            if (jointIdx < 5) {
                joint_positions[jointIdx] = content.substring(startIdx, i).toFloat();
                jointIdx++;
            }
            startIdx = i + 1;
        }
    }
    
    if (jointIdx == 5) {
        Serial.print("[Teensy] ✓ Parsed: [");
        for (int i = 0; i < 5; i++) {
            Serial.print(joint_positions[i], 3);
            if (i < 4) Serial.print(", ");
        }
        Serial.println("]");
        initialized = true;
    } else {
        Serial.println("[Teensy] ✗ Parse error");
    }
}

void sendPositionFeedback() {
    Serial.print("<");
    for (int i = 0; i < 5; i++) {
        Serial.print(joint_positions[i], 4);
        if (i < 4) Serial.print(",");
    }
    Serial.println(">");
}
