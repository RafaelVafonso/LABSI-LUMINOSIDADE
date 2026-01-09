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

// Variáveis de Estado
String modo = "--";
String setpoint = "--";
String lux = "--";
// SEPARAR MOTORES E TICKS
String motor1 = "--"; 
String motor2 = "--";
String ticks1 = "0";
String ticks2 = "0";

// memória Flash (PROGMEM)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>LABSI Dashboard</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: sans-serif; text-align: center; background: #1a1a1a; color: #fff; margin:0; padding:10px; }
    h1 { color: #FFC107; margin-bottom: 5px; }
    
    /* Cartões */
    .card { background: #2d2d2d; padding: 15px; border-radius: 12px; margin-bottom: 15px; box-shadow: 0 4px 6px rgba(0,0,0,0.3); }
    .section-title { color: #888; font-size: 12px; font-weight: bold; letter-spacing: 1px; margin-bottom: 10px; text-transform: uppercase; border-bottom: 1px solid #444; padding-bottom: 5px; }
    
    /* Valores */
    .val { font-size: 22px; font-weight: bold; color: #00E676; }
    .lbl { color: #aaa; font-size: 12px; margin-top: 2px; }
    .row { display: flex; justify-content: space-around; margin-bottom: 5px; }
    
    /* Barras de Progresso */
    .progress-container { margin-top:15px; text-align:left; }
    .progress-bg { width: 100%; background-color: #444; border-radius: 4px; height: 12px; margin-top: 5px; }
    .progress-bar { height: 100%; background-color: #2196F3; width: 0%; border-radius: 4px; transition: width 0.3s; }
    
    /* Botões */
    button { width: 100%; height: 50px; margin: 5px 0; font-size: 18px; border: none; border-radius: 8px; color: white; cursor: pointer; font-weight: bold; transition: 0.1s; }
    button:active { transform: scale(0.96); opacity: 0.8; }
    
    .b-mode { background: #2196F3; width: 48%; }
    .b-up   { background: #FFC107; color: black; width: 48%; }
    .b-down { background: #FF9800; width: 48%; }
    .b-open { background: #4CAF50; } 
    .b-close{ background: #F44336; } 
    .b-stop { background: #9E9E9E; }
    
    .btn-row { display: flex; justify-content: space-between; gap: 5px; }
  </style>
</head>
<body>
  <h1>LABSI CONTROL</h1>
  
  <!-- DASHBOARD -->
  <div class="card">
    <div class="section-title">Monitorizacao</div>
    <div class="row">
      <div><div class="val" id="t_mode">--</div><div class="lbl">MODO</div></div>
      <!-- ID Corrigido para Motor 2 -->
      <div><div class="val" id="t_mot1" style="color:#FFF; font-size:18px">--</div><div class="lbl">MOTOR 1</div></div>
      <div><div class="val" id="t_mot2" style="color:#FFF; font-size:18px">--</div><div class="lbl">MOTOR 2</div></div>
    </div>
    <div style="height:10px"></div>
    <div class="row">
      <div><div class="val" id="t_lux" style="color:#FFC107">--</div><div class="lbl">LUX ATUAL</div></div>
      <div><div class="val" id="t_set" style="color:#FFC107">--</div><div class="lbl">SETPOINT</div></div>
    </div>
    
    <!-- BARRA PERSIANA 1 -->
    <div class="progress-container">
      <span class="lbl" style="margin-left:5px">POSIÇÃO PERSIANA 1 (<span id="t_ticks1">0</span>)</span>
      <div class="progress-bg"><div class="progress-bar" id="p_bar1"></div></div>
    </div>

    <!-- BARRA PERSIANA 2 -->
    <div class="progress-container">
      <span class="lbl" style="margin-left:5px">POSIÇÃO PERSIANA 2 (<span id="t_ticks2">0</span>)</span>
      <div class="progress-bg"><div class="progress-bar" id="p_bar2" style="background-color:#9C27B0"></div></div>
    </div>
  </div>

  <!-- CONTROLOS -->
  <div class="card">
    <div class="section-title">Modo de Operacao</div>
    <div class="btn-row">
      <button class="b-mode" onclick="s('A')">AUTO</button>
      <button class="b-mode" style="background:#5c6bc0" onclick="s('M')">MANUAL</button>
    </div>
  </div>
  
  <div class="card">
    <div class="section-title">Ajuste (Setpoint / LED)</div>
    <div class="btn-row">
      <button class="b-down" onclick="s('D')">MENOS (-)</button>
      <button class="b-up" onclick="s('U')">MAIS (+)</button>
    </div>
  </div>
  
  <div class="card">
    <div class="section-title">Persiana 1 (Manual)</div>
    <div class="btn-row">
       <button class="b-open" style="width:32%" onclick="s('O')">SOBE</button>
       <button class="b-stop" style="width:32%" onclick="s('S')">PARA</button>
       <button class="b-close" style="width:32%" onclick="s('C')">DESCE</button>
    </div>
  </div>
  
  <div class="card">
    <div class="section-title">Persiana 2 (Manual)</div>
    <div class="btn-row">
       <button class="b-open" style="width:32%" onclick="s('X')">SOBE</button>
       <button class="b-stop" style="width:32%" onclick="s('P')">PARA</button>
       <button class="b-close" style="width:32%" onclick="s('F')">DESCE</button>
    </div>
  </div>

  <script>
    function s(c) { fetch("/cmd?v="+c); }
    
    function formatMotor(code) {
        if(code == 'A') return "ABRIR";
        if(code == 'F') return "FECHAR";
        return "PARADO";
    }

    setInterval(function() {
      fetch("/status").then(r => r.json()).then(d => {
        document.getElementById("t_mode").innerText = (d.m == 'A') ? "AUTO" : "MANUAL";
        document.getElementById("t_mode").style.color = (d.m == 'A') ? "#00E676" : "#2196F3";

        // Atualiza Estados Motores
        document.getElementById("t_mot1").innerText = formatMotor(d.mt1);
        document.getElementById("t_mot2").innerText = formatMotor(d.mt2);

        document.getElementById("t_lux").innerText = d.l;
        document.getElementById("t_set").innerText = d.s;
        
        // Atualiza P1
        document.getElementById("t_ticks1").innerText = d.t1;
        let pct1 = (d.t1 / 6318) * 100; if(pct1 > 100) pct1 = 100;
        document.getElementById("p_bar1").style.width = pct1 + "%";

        // Atualiza P2
        document.getElementById("t_ticks2").innerText = d.t2;
        let pct2 = (d.t2 / 3400) * 100; if(pct2 > 100) pct2 = 100;
        document.getElementById("p_bar2").style.width = pct2 + "%";
      });
    }, 500);
  </script>
</body>
</html>
)rawliteral";

// --- HANDLERS DO SERVIDOR ---

// Serve a página principal
void handleRoot() {
  server.send(200, "text/html", index_html);
}
void handleStatus() { 
  // Cria JSON atualizado para 2 persianas
  String json = "{\"m\":\"" + modo + "\",\"s\":\"" + setpoint + "\",\"l\":\"" + lux + 
                "\",\"mt1\":\"" + motor1 + "\",\"mt2\":\"" + motor2 + 
                "\",\"t1\":\"" + ticks1 + "\",\"t2\":\"" + ticks2 + "\"}";
  server.send(200, "application/json", json);
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
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  WiFi.softAP(ssid, password);
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/cmd", handleCommand);
  server.on("/status", handleStatus);
  server.begin();
}

void loop() {
  server.handleClient();
  
  // LER DADOS DO ATmega (Formato: D:A,300,150,P,P,1200,1000\n)
  if (Serial2.available()) {
    String line = Serial2.readStringUntil('\n');
    
if (line.startsWith("D:")) {
    line = line.substring(2); // remove "D:"
    line.trim();

    int idx = 0;
    String parts[7];

    while (line.length() > 0 && idx < 7) {
        int comma = line.indexOf(',');
        if (comma < 0) {
            parts[idx++] = line;
            break;
        }
        parts[idx++] = line.substring(0, comma);
        line = line.substring(comma + 1);
    }

    modo    = parts[0];
    setpoint= parts[1];
    lux     = parts[2];
    motor1  = parts[3];
    motor2  = parts[4];
    ticks1  = parts[5];
    ticks2  = parts[6];
}

  }
}