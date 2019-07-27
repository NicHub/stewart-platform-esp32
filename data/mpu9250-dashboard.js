"use strict"
var scene;
var camera;
var renderer;
var axis3d;
var scene3dcontainer = document.getElementById("scene3dcontainer");
var infoPanel = document.getElementById("infoPanel");

{
    console.log("### START ###");
    CubeBegin();
    getIP(webSocketHandle);
}

function render() {
    requestAnimationFrame(render);
    renderer.render(scene, camera);
}

window.onresize = function () {
    CubeBegin()
};

function CubeBegin() {
    scene3dcontainer.innerHTML = "";

    scene = new THREE.Scene();

    if (false) {
        camera = new THREE.PerspectiveCamera(
            650,
            scene3dcontainer.clientWidth / scene3dcontainer.clientHeight,
            0.1,
            1000
        );
    } else {
        var zoomFact = 4;
        camera = new THREE.OrthographicCamera(
            scene3dcontainer.clientWidth / - zoomFact,
            scene3dcontainer.clientWidth / zoomFact,
            scene3dcontainer.clientHeight / zoomFact,
            scene3dcontainer.clientHeight / - zoomFact,
            1,
            1000);
    }

    renderer = new THREE.WebGLRenderer();
    renderer.setSize(scene3dcontainer.clientWidth, scene3dcontainer.clientHeight);
    scene3dcontainer.appendChild(renderer.domElement);

    var drawWireframe = false;

    var cube_Geometry = new THREE.BoxGeometry(30, 30, 30);
    var cubeMaterials = [
        new THREE.MeshBasicMaterial({ color: 0xfe4365 }),
        new THREE.MeshBasicMaterial({ color: 0xfc9d9a }),
        new THREE.MeshBasicMaterial({ color: 0xf9cdad }),
        new THREE.MeshBasicMaterial({ color: 0xc8cba9 }),
        new THREE.MeshBasicMaterial({ color: 0x83af98 }),
        new THREE.MeshBasicMaterial({ color: 0xe5fcc2 })
    ];
    var cube_Mesh = new THREE.Mesh(cube_Geometry, cubeMaterials);

    var arrow_A_Geometry = new THREE.ConeGeometry(15, 30, 32);
    var arrow_A_Material = new THREE.MeshBasicMaterial({ color: new THREE.Color("orangered"), wireframe: drawWireframe });
    var arrow_A_Mesh = new THREE.Mesh(arrow_A_Geometry, arrow_A_Material);
    arrow_A_Mesh.rotation.z = Math.PI / 180 * 90;
    arrow_A_Mesh.position.x = -75;

    var cyl_A_Geometry = new THREE.CylinderGeometry(5, 5, 60, 32);
    var cyl_A_Material = new THREE.MeshBasicMaterial({ color: new THREE.Color("hsl(0, 0%, 90%)"), wireframe: drawWireframe });
    var cyl_A_Mesh = new THREE.Mesh(cyl_A_Geometry, cyl_A_Material);
    cyl_A_Mesh.rotation.z = Math.PI / 180 * 90;
    cyl_A_Mesh.position.x = -30;

    var arrow_B_Geometry = new THREE.ConeGeometry(15, 30, 32);
    var arrow_B_Material = new THREE.MeshBasicMaterial({ color: new THREE.Color("mediumseagreen"), wireframe: drawWireframe });
    var arrow_B_Mesh = new THREE.Mesh(arrow_B_Geometry, arrow_B_Material);
    arrow_B_Mesh.rotation.z = Math.PI / 180 * 0;
    arrow_B_Mesh.position.y = 75;

    var cyl_B_Geometry = new THREE.CylinderGeometry(5, 5, 60, 32);
    var cyl_B_Material = new THREE.MeshBasicMaterial({ color: new THREE.Color("hsl(0, 0%, 90%)"), wireframe: drawWireframe });
    var cyl_B_Mesh = new THREE.Mesh(cyl_B_Geometry, cyl_B_Material);
    cyl_B_Mesh.rotation.z = Math.PI / 180 * 0;
    cyl_B_Mesh.position.y = 30;

    var arrow_C_Geometry = new THREE.ConeGeometry(15, 30, 32);
    var arrow_C_Material = new THREE.MeshBasicMaterial({ color: new THREE.Color("dodgerblue"), wireframe: drawWireframe });
    var arrow_C_Mesh = new THREE.Mesh(arrow_C_Geometry, arrow_C_Material);
    arrow_C_Mesh.rotation.x = Math.PI / 180 * 270;
    arrow_C_Mesh.position.z = -75;

    var cyl_C_Geometry = new THREE.CylinderGeometry(5, 5, 60, 32);
    var cyl_C_Material = new THREE.MeshBasicMaterial({ color: new THREE.Color("hsl(0, 0%, 90%)"), wireframe: drawWireframe });
    var cyl_C_Mesh = new THREE.Mesh(cyl_C_Geometry, cyl_C_Material);
    cyl_C_Mesh.rotation.x = Math.PI / 180 * 270;
    cyl_C_Mesh.position.z = -30;

    axis3d = new THREE.Group();
    axis3d.add(cube_Mesh);
    axis3d.add(arrow_A_Mesh);
    axis3d.add(cyl_A_Mesh);
    axis3d.add(arrow_B_Mesh);
    axis3d.add(cyl_B_Mesh);
    axis3d.add(arrow_C_Mesh);
    axis3d.add(cyl_C_Mesh);
    scene.add(axis3d);
    // camera.position.set(100, -100, -100);
    // camera.up = new THREE.Vector3(0, 0, -1);
    // camera.lookAt(new THREE.Vector3(0, 0, 0));
    camera.position.set(-100, -100, 100);
    camera.up = new THREE.Vector3(0, 0, 1);
    camera.lookAt(new THREE.Vector3(0, 0, 0));
    render();
}

function getIP(callback) {
    // Check if the HTML file is served from the ESP
    // or from a development server. This is done by
    // checking if the file `info.json` is present on
    // the server. If it is present, then we are on
    // the ESP. If we are on a development server, the
    // IP address of the ESP must be set manually below.
    var file_url = location.origin + "/info.json";
    var xhr = new XMLHttpRequest();
    xhr.open("GET", file_url, true);
    xhr.send();
    xhr.onload = function () {
        var ip;
        var ans = xhr.status;
        if (ans === 200) {
            // We are on the ESP.
            ip = location.hostname;
            document.title = "ESP32—" + document.title;
        }
        else {
            // We are on the development server.
            // Manually set ESP IP address.
            // ip = "192.168.1.113";
            ip = "192.168.2.16";
            document.title = "DEV[" + ans + "]—" + document.title;
        }
        console.log("ip = " + ip);
        callback(ip);
    }
}

function webSocketHandle(ip) {
    if (! "WebSocket" in window) {
        alert("WebSocket is not supported by your browser!");
        return;
    }

    var ws = new WebSocket("ws://" + ip + "/ws", ["arduino"]);
    var T1 = +new Date();
    var deltaTmax = 0;


    // Dials
    var dialA = document.getElementById("dialA");
    var dialB = document.getElementById("dialB");
    var dialC = document.getElementById("dialC");
    var dialAangle = document.getElementById("dialAangle");
    var dialBangle = document.getElementById("dialBangle");
    var dialCangle = document.getElementById("dialCangle");

    var transfA = "rotate(";
    var transfB = "rotate(";
    var transfC = "rotate(";
    // End Dials


    ws.onmessage = function (evt) {
        var T2 = +new Date();
        var deltaTms = T2 - T1;
        deltaTmax = (deltaTms > deltaTmax) ? deltaTms : deltaTmax;
        var refreshRate = 1000 / deltaTms;
        T1 = T2;

        var data = JSON.parse(evt.data);
        infoPanel.innerHTML = "<pre>" + evt.data + "</pre><pre>Refresh rate = "
            + refreshRate.toFixed() + " Hz (ΔT = " + deltaTms.toFixed() + " ms | ΔTmax = " + deltaTmax + "ms)</pre>";

        if (!data.hasOwnProperty("quat") || !data.hasOwnProperty("euler")) {
            return;
        }
        var quat1 = new THREE.Quaternion(data.quat.qx, data.quat.qy, data.quat.qz, data.quat.qw);
        var quat2 = new THREE.Quaternion(1, 0, 0, 0);

        axis3d.quaternion.multiplyQuaternions(quat1, quat2);

        // Dials
        var ALGO = "from_esp";
        if (ALGO === "threejs") {
            var quat1 = new THREE.Quaternion(
                data.quat.qx, data.quat.qy,
                data.quat.qz, data.quat.qw);
            // var eulerOrder = "XYZ";    // x  OK, y NOK, z  OK
            // var eulerOrder = "ZXY";    // x NOK, y  OK, z  OK
            // var eulerOrder = "YZX";    // x  OK, y  OK, z NOK
            var eulerOrder = "YXZ";    // x NOK, y  OK, z  OK
            var eViaQ1 = new THREE.Euler().setFromQuaternion(quat1, eulerOrder);
            var mViaQ1 = new THREE.Matrix4().makeRotationFromQuaternion(quat1);
            var eViaMViaQ1 = new THREE.Euler().setFromRotationMatrix(mViaQ1, eulerOrder);

            // console.log(eViaQ1);
            // console.log(eViaMViaQ1);
            // console.log(eViaQ1);
            console.log(quat1);
            var RAD_TO_DEG = 57.295779513082320876798154814105;
            data.euler.eA = eViaMViaQ1._x * RAD_TO_DEG;
            data.euler.eB = eViaMViaQ1._y * RAD_TO_DEG;
            data.euler.eC = eViaMViaQ1._z * RAD_TO_DEG;

        } else if (ALGO === "from_esp") {
        }

        dialA.setAttribute("transform", transfA + data.euler.eA + ")");
        dialB.setAttribute("transform", transfB + data.euler.eB + ")");
        dialC.setAttribute("transform", transfC + data.euler.eC + ")");
        dialAangle.textContent = Number(data.euler.eA).toFixed(2) + "°";
        dialBangle.textContent = Number(data.euler.eB).toFixed(2) + "°";
        dialCangle.textContent = Number(data.euler.eC).toFixed(2) + "°";

        // End Dials
    };

    ws.onclose = function () {
        infoPanel.innerHTML = "<pre>WebSocket connection is closed</pre>";
    };
}
