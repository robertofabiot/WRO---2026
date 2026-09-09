from flask import Flask, render_template_string, request, jsonify
import pyautogui
import socket

app = Flask(__name__)

# Configurar pyautogui para que sea rápido
pyautogui.PAUSE = 0.01

# Diseño visual muy moderno para el celular
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Control Remoto Robot WRO</title>
    <style>
        :root {
            --bg: #0f172a;
            --panel: #1e293b;
            --primary: #3b82f6;
            --primary-active: #2563eb;
            --text: #f8fafc;
            --accent-green: #10b981;
            --accent-red: #ef4444;
            --accent-orange: #f59e0b;
        }
        
        * {
            box-sizing: border-box;
            user-select: none;
            -webkit-user-select: none;
        }

        body {
            margin: 0;
            padding: 20px;
            background-color: var(--bg);
            color: var(--text);
            font-family: 'Segoe UI', Roboto, Helvetica, sans-serif;
            height: 100vh;
            display: flex;
            flex-direction: column;
            overflow: hidden;
            gap: 15px;
        }

        .header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            background: var(--panel);
            padding: 10px 20px;
            border-radius: 15px;
            box-shadow: 0 4px 15px rgba(0,0,0,0.3);
        }

        .header h1 {
            font-size: 1.2rem;
            margin: 0;
            color: var(--text);
            letter-spacing: 1px;
            display: flex;
            align-items: center;
            gap: 10px;
        }

        .sys-controls {
            display: flex;
            gap: 10px;
        }

        .sys-btn {
            background: var(--primary);
            border: none;
            border-radius: 8px;
            color: white;
            padding: 8px 15px;
            font-weight: 600;
            cursor: pointer;
            display: flex;
            align-items: center;
            gap: 5px;
            transition: 0.1s;
        }
        .sys-btn:active { transform: scale(0.95); }
        .sys-btn.start { background: var(--accent-green); }
        .sys-btn.stop { background: var(--accent-red); }

        .container {
            display: flex;
            flex: 1;
            gap: 15px;
            justify-content: space-between;
        }

        .panel {
            background: var(--panel);
            border-radius: 20px;
            padding: 15px;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            box-shadow: 0 10px 25px rgba(0,0,0,0.5);
            flex: 1;
        }

        .panel-title {
            font-size: 0.85rem;
            color: #94a3b8;
            margin-bottom: 15px;
            text-transform: uppercase;
            letter-spacing: 1px;
        }

        /* D-PAD */
        .dpad {
            display: grid;
            grid-template-columns: repeat(3, 60px);
            grid-template-rows: repeat(3, 60px);
            gap: 8px;
        }

        .btn {
            background: #334155;
            border: none;
            border-radius: 15px;
            color: white;
            display: flex;
            align-items: center;
            justify-content: center;
            cursor: pointer;
            transition: transform 0.1s, background 0.1s;
            box-shadow: 0 4px 6px rgba(0,0,0,0.3);
            touch-action: manipulation;
        }

        .btn:active {
            transform: scale(0.92);
            background: var(--primary-active);
        }

        .btn svg { width: 28px; height: 28px; }

        .btn-up { grid-column: 2; grid-row: 1; }
        .btn-left { grid-column: 1; grid-row: 2; }
        .btn-stop { grid-column: 2; grid-row: 2; background: var(--accent-red); }
        .btn-stop:active { background: #b91c1c; }
        .btn-right { grid-column: 3; grid-row: 2; }
        .btn-down { grid-column: 2; grid-row: 3; }

        /* Botones de acción (Garras) */
        .actions-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 12px;
            width: 100%;
        }

        .action-btn {
            background: var(--primary);
            padding: 12px 5px;
            border-radius: 12px;
            font-size: 0.9rem;
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 8px;
            font-weight: 500;
        }
        
        .action-btn.green { background: var(--accent-green); }
        .action-btn.orange { background: var(--accent-orange); }
        
        .action-btn svg { width: 24px; height: 24px; }
    </style>
</head>
<body>
    <div class="header">
        <h1>
            <svg viewBox="0 0 24 24" width="24" height="24" stroke="currentColor" stroke-width="2" fill="none"><rect x="2" y="14" width="20" height="8" rx="2" ry="2"></rect><rect x="6" y="2" width="12" height="8" rx="2" ry="2"></rect><line x1="12" y1="14" x2="12" y2="10"></line></svg>
            WRO 2026 Controller
        </h1>
        <div class="sys-controls">
            <button class="sys-btn start" onpointerdown="sendKey('f5')">
                <svg viewBox="0 0 24 24" width="16" height="16" stroke="currentColor" stroke-width="2" fill="none"><polygon points="5 3 19 12 5 21 5 3"></polygon></svg>
                F5
            </button>
            <button class="sys-btn stop" onpointerdown="sendKey('f6')">
                <svg viewBox="0 0 24 24" width="16" height="16" stroke="currentColor" stroke-width="2" fill="none"><rect x="3" y="3" width="18" height="18" rx="2" ry="2"></rect></svg>
                F6
            </button>
        </div>
    </div>
    
    <div class="container">
        <!-- Chasis Control -->
        <div class="panel">
            <div class="panel-title">Chasis</div>
            <div class="dpad">
                <button class="btn btn-up" onpointerdown="sendKey('w')">
                    <svg viewBox="0 0 24 24" stroke="currentColor" stroke-width="2.5" fill="none" stroke-linecap="round" stroke-linejoin="round"><polyline points="18 15 12 9 6 15"></polyline></svg>
                </button>
                <button class="btn btn-left" onpointerdown="sendKey('a')">
                    <svg viewBox="0 0 24 24" stroke="currentColor" stroke-width="2.5" fill="none" stroke-linecap="round" stroke-linejoin="round"><polyline points="15 18 9 12 15 6"></polyline></svg>
                </button>
                <button class="btn btn-stop" onpointerdown="sendKey(' ')">
                    <svg viewBox="0 0 24 24" stroke="currentColor" stroke-width="2.5" fill="none" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="10"></circle><line x1="4.93" y1="4.93" x2="19.07" y2="19.07"></line></svg>
                </button>
                <button class="btn btn-right" onpointerdown="sendKey('d')">
                    <svg viewBox="0 0 24 24" stroke="currentColor" stroke-width="2.5" fill="none" stroke-linecap="round" stroke-linejoin="round"><polyline points="9 18 15 12 9 6"></polyline></svg>
                </button>
                <button class="btn btn-down" onpointerdown="sendKey('s')">
                    <svg viewBox="0 0 24 24" stroke="currentColor" stroke-width="2.5" fill="none" stroke-linecap="round" stroke-linejoin="round"><polyline points="6 9 12 15 18 9"></polyline></svg>
                </button>
            </div>
        </div>

        <!-- Garras Control -->
        <div class="panel">
            <div class="panel-title">Mecanismos</div>
            <div class="actions-grid">
                <button class="btn action-btn green" onpointerdown="sendKey('i')">
                    <svg viewBox="0 0 24 24" stroke="currentColor" stroke-width="2" fill="none" stroke-linecap="round" stroke-linejoin="round"><polyline points="17 11 12 6 7 11"></polyline><polyline points="17 18 12 13 7 18"></polyline></svg>
                    Subir Ele.
                </button>
                <button class="btn action-btn green" onpointerdown="sendKey('k')">
                    <svg viewBox="0 0 24 24" stroke="currentColor" stroke-width="2" fill="none" stroke-linecap="round" stroke-linejoin="round"><polyline points="7 13 12 18 17 13"></polyline><polyline points="7 6 12 11 17 6"></polyline></svg>
                    Bajar Ele.
                </button>
                
                <button class="btn action-btn" onpointerdown="sendKey('j')">
                    <svg viewBox="0 0 24 24" stroke="currentColor" stroke-width="2" fill="none" stroke-linecap="round" stroke-linejoin="round"><rect x="3" y="11" width="18" height="11" rx="2" ry="2"></rect><path d="M7 11V7a5 5 0 0 1 9.9-1"></path></svg>
                    Abrir Pinza
                </button>
                <button class="btn action-btn" onpointerdown="sendKey('l')">
                    <svg viewBox="0 0 24 24" stroke="currentColor" stroke-width="2" fill="none" stroke-linecap="round" stroke-linejoin="round"><rect x="3" y="11" width="18" height="11" rx="2" ry="2"></rect><path d="M7 11V7a5 5 0 0 1 10 0v4"></path></svg>
                    Cerrar Pinza
                </button>

                <button class="btn action-btn orange" onpointerdown="sendKey('u')">
                    <svg viewBox="0 0 24 24" stroke="currentColor" stroke-width="2" fill="none" stroke-linecap="round" stroke-linejoin="round"><line x1="12" y1="19" x2="12" y2="5"></line><polyline points="5 12 12 5 19 12"></polyline></svg>
                    Subir Tras.
                </button>
                <button class="btn action-btn orange" onpointerdown="sendKey('o')">
                    <svg viewBox="0 0 24 24" stroke="currentColor" stroke-width="2" fill="none" stroke-linecap="round" stroke-linejoin="round"><line x1="12" y1="5" x2="12" y2="19"></line><polyline points="19 12 12 19 5 12"></polyline></svg>
                    Bajar Tras.
                </button>
            </div>
        </div>
    </div>

    <script>
        function sendKey(key) {
            if (navigator.vibrate) {
                navigator.vibrate(40);
            }
            fetch('/press', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ key: key })
            });
        }
        window.oncontextmenu = function(e) {
            e.preventDefault();
            e.stopPropagation();
            return false;
        };
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/press', methods=['POST'])
def press():
    data = request.json
    key = data.get('key')
    if key:
        print(f"Tecla recibida desde el celular: {key}")
        # pyautogui.press simula que tu teclado presionó la tecla en la PC
        pyautogui.press(key)
    return jsonify(success=True)

def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # No necesita conectarse realmente
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

if __name__ == '__main__':
    ip_local = get_ip()
    print("="*50)
    print("🚀 SERVIDOR DEL CONTROL REMOTO VISUAL INICIADO 🚀")
    print("="*50)
    print(f"1. Asegúrate de que tu celular y PC estén en el mismo WiFi.")
    print(f"2. Abre el navegador en tu celular y entra a esta dirección:")
    print(f"\n      http://{ip_local}:5000\n")
    print("3. Corre tu script original 'control_remoto.py' en Pybricks.")
    print("4. IMPORTANTE: Haz clic en la terminal de Pybricks para que quede seleccionada.")
    print("5. ¡Usa los botones en tu celular! La PC escribirá por ti.")
    print("="*50)
    
    # Inicia el servidor Flask
    app.run(host='0.0.0.0', port=5000)
