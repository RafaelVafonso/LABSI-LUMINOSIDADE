#include <WiFi.h>
#include <WebServer.h>

// --- CONFIGURAÇÃO DA TUA REDE (MODO AP) ---
const char* ssid = "LABSI_CONTROLO";   // Nome da rede que vai aparecer no telemóvel
const char* password = "LABSIUY719"; // Senha (min 8 caracteres)

WebServer server(80);

// UART para ATmega (RX=16, TX=17)
// Nota: O pino 16 (RX2) liga ao TX do ATmega (com divisor de tensão!)
// Nota: O pino 17 (TX2) liga ao RX do ATmega (direto)
#define RXD2 16
#define TXD2 17

// --- PÁGINA WEB (HTML/CSS/JS) ---
// Guardada na memória Flash (PROGMEM)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>LABSI IoT</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: sans-serif; text-align: center; background: #222; color: #fff; margin:0; padding:20px; }
    h1 { color: #FFC107; }
    button { width: 100%; height: 60px; margin: 10px 0; font-size: 20px; border: none; border-radius: 10px; color: white; cursor: pointer; font-weight: bold; }
    button:active { transform: scale(0.98); }
    .b-mode { background: #2196F3; }
    .b-up { background: #FFC107; color: black; }
    .b-down { background: #FF9800; }
    .b-open { background: #4CAF50; }
    .b-close { background: #F44336; }
    .b-stop { background: #9E9E9E; }
    .section { margin-top: 20px; padding: 10px; background: #333; border-radius: 10px; }
  </style>
</head>
<body>
  <h1>LABSI CONTROLo</h1>
  
  <div class="section">
    <div>MODO DE OPERACAO</div>
    <button class="b-mode" onclick="s('A')">AUTOMATICO</button>
    <button class="b-mode" onclick="s('M')">MANUAL</button>
  </div>
  
  <div class="section">
    <div>AJUSTE (Setpoint / LED)</div>
    <button class="b-up" onclick="s('U')">AUMENTAR (+)</button>
    <button class="b-down" onclick="s('D')">DIMINUIR (-)</button>
  </div>
  
  <div class="section">
    <div>PERSIANA (Manual)</div>
    <button class="b-open" onclick="s('O')">ABRIR</button>
    <button class="b-stop" onclick="s('S')">PARAR</button>
    <button class="b-close" onclick="s('C')">FECHAR</button>
  </div>

  <script>
    // Função JavaScript para enviar comando sem recarregar a página
    function s(c) { 
      fetch("/cmd?v="+c).then(r => {
        console.log("Comando enviado: " + c);
      }).catch(e => console.log(e));
    }
  </script>
</body>
</html>
)rawliteral";

// --- HANDLERS DO SERVIDOR ---

// Serve a página principal
void handleRoot() {
  server.send(200, "text/html", index_html);
}

// Recebe comandos via AJAX (ex: /cmd?v=A)
void handleCommand() {
  if (server.hasArg("v")) {
    char c = server.arg("v").charAt(0);
    
    // 1. Envia para o ATmega via Serial2
    Serial2.write(c); 
    
    // 2. Debug no PC (Opcional)
    Serial.print("Web -> ATmega: ");
    Serial.println(c);
  }
  server.send(200, "text/plain", "OK");
}

void setup() {
  // Serial de Debug (USB para PC)
  Serial.begin(115200);
  
  // Serial de Comunicação com ATmega (Baudrate deve ser igual ao do ATmega)
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // Configurar Ponto de Acesso (AP)
  Serial.println("A configurar Access Point...");
  WiFi.softAP(ssid, password);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Rede criada com sucesso!");
  Serial.print(" Nome: "); Serial.println(ssid);
  Serial.print(" IP para aceder: http://"); Serial.println(IP);

  // Rotas Web
  server.on("/", handleRoot);
  server.on("/cmd", handleCommand);
  
  server.begin();
  Serial.println("Servidor HTTP iniciado");
}

void loop() {
  // 1. Mantém o servidor web ativo para receber pedidos do telemóvel
  server.handleClient();
  
  // 2. Lê dados vindos do ATmega e mostra no Monitor Serial do PC (Pass-through)
  // Isto é muito útil para debug se o ATmega enviar mensagens de volta
  if (Serial2.available()) {
    char c = Serial2.read();
    Serial.write(c); 
  }
}