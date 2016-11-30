const char *indexHtml PROGMEM = R"(
<!doctype html>

<html>
    <head>
        <meta charset='utf-8' />
        <meta name='viewport' content='width=device-width, initial-scale=1' />

        <title>ESPway</title>

        <style>
            html {
                font-size: 32px;
            }

            body {
                font-family: sans-serif;
                text-align: center;
                margin: 0 auto;
            }

            button {
                font-size: 2rem;
                width: 4rem;
                height: 3rem;
            }

            #cube {
                transform-style: preserve-3d;
                position: absolute;
                width: 200px;
                height: 200px;
                margin: 0;
            }

            .square {
                width: 200px;
                height: 200px;
                position: absolute;
                margin: 0 auto;
            }

            #square1A, #square1B {
                background-color: blue;
            }

            #square2A, #square2B {
                background-color: red;
            }

            #square3A, #square3B {
                background-color: green;
            }

            #square1A {
                transform: translate3d(0, 0, 100px);
            }
            #square1B {
                transform: translate3d(0, 0, -100px);
            }

            #square2A {
                transform: rotateY(90deg) translate3d(0, 0, -100px);
            }
            #square2B {
                transform: rotateY(90deg) translate3d(0, 0, 100px);
            }

            #square3A {
                transform: rotateX(90deg) translate3d(0, 0, -100px);
            }
            #square3B {
                transform: rotateX(90deg) translate3d(0, 0, 100px);
            }

            #container {
                margin: 100px auto;
                width: 200px;
                height: 200px;
            }
        </style>
    </head>

    <body>
        <div id='container'>
            <div id='cube'>
                <div class='square' id='square1A'></div>
                <div class='square' id='square2A'></div>
                <div class='square' id='square3A'></div>
                <div class='square' id='square1B'></div>
                <div class='square' id='square2B'></div>
                <div class='square' id='square3B'></div>
            </div>
        </div>

        <script>
            (function() {
                'use strict'

                let byId = id => document.getElementById(id)
                let cube = byId('cube')
                let scale = 1 / 1000.0

                function main() {
                    let ws = new WebSocket('ws://192.168.4.1/ws')
                    ws.binaryType = 'arraybuffer'

                    ws.addEventListener('message', e => {
                        if (e.data.byteLength === 8) {
                            let arr = new Int16Array(e.data)
                            let q0 = scale * arr[0],
                                q1 = scale * arr[1],
                                q2 = scale * arr[2],
                                q3 = scale * arr[3]
                            let angle = 2.0 * Math.acos(q0)
                            let sinHalfAngle = Math.sin(angle / 2)
                            let a1 = q0*q0 + q1*q1 - q2*q2 - q3*q3,
                                a2 = 2 * (q1*q2 - q0*q3),
                                a3 = 2 * (q1*q3 + q0*q2),
                                b1 = 2 * (q1*q2 + q0*q3),
                                b2 = q0*q0 - q1*q1 + q2*q2 - q3*q3,
                                b3 = 2 * (q2*q3 - q0*q1),
                                c1 = 2 * (q1*q3 - q0*q2),
                                c2 = 2 * (q2*q3 + q0*q1),
                                c3 = q0*q0 - q1*q1 - q2*q2 + q3*q3
                            let transform = 'matrix3d(' +
                                a1 + ',' + b1 + ',' + c1 + ',0,' +
                                a2 + ',' + b2 + ',' + c2 + ',0,' +
                                a3 + ',' + b3 + ',' + c3 + ',0,' +
                                '0,0,0,1)'
                            cube.style.transform = transform
                        }
                    })

                    let sendBytes = bytes => ws.send((new Uint8Array(bytes)).buffer)
                }

                window.addEventListener('load', main)
            })()
        </script>
    </body>
</html>

)";
