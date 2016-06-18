"use strict";



//////////////////////////////////////////////////////
// CLASSES
//
function Point(x, y) {
    this.x = x;
    this.y = y;
}
Point.prototype.equals = function (other) {
    return other.x == this.x && other.y == this.y;
};
Point.prototype.copy = function () {
    return new Point(this.x, this.y);
};

function DijkstraNodeInfo() {
    this.distToStart = Infinity;
    this.last = null;
}

function Node(pos) {
    this.pos = pos.copy();
    this.neighbours = [];
    this.isWall = false;
    this.aStarInfo = null;
    this.dijkstraInfo = null;
}
//////////////////////////////////////////////////////



//////////////////////////////////////////////////////
// SHARED GLOBALS
//
var aStarCanvas = document.getElementById("aStarCanvas");
var aStarCtx = aStarCanvas.getContext("2d");
var dijkstraCanvas = document.getElementById("dijkstraCanvas");
var dijkstraCtx = dijkstraCanvas.getContext("2d");
var width = aStarCanvas.width;
var height = aStarCanvas.height;
var nodeSize = 25;
var numNodesH = width / nodeSize;
var numNodesV = height / nodeSize;
var nodes = null;
var fieldType = document.getElementById("fieldType");
var startButton = document.getElementById("start");
var resetButton = document.getElementById("reset");
var stepButton = document.getElementById("step");
var finishButton = document.getElementById("finish");
var running = false;
var startNode = null;
var goalNode = null;
//////////////////////////////////////////////////////



//////////////////////////////////////////////////////
// ASTAR STUFF
//
function AStarNodeInfo() {
    this.f = 0;
    this.g = 0;
    this.last = null;
}

var aStarOpenList = [];
var aStarClosedList = [];
var aStarSucceeded = false;
var aStarRunning = false;
var aStarPath = [];

function aStarRender() {
    aStarCtx.fillStyle = "#4499FF";
    for (var i = 0; i < aStarOpenList.length; i++) {
        aStarCtx.fillRect(aStarOpenList[i].pos.x * nodeSize, 
                aStarOpenList[i].pos.y * nodeSize, 
                nodeSize, nodeSize);
    }
    
    aStarCtx.fillStyle = "#0000FF";
    for (var i = 0; i < aStarClosedList.length; i++) {
        aStarCtx.fillRect(aStarClosedList[i].pos.x * nodeSize, 
                aStarClosedList[i].pos.y * nodeSize, 
                nodeSize, nodeSize);
    }
    
    if(aStarSucceeded == true) {
        aStarCtx.fillStyle = "#FFFF00";
        for (var i = 0; i < aStarPath.length; i++) {
            aStarCtx.fillRect(aStarPath[i].pos.x * nodeSize, 
                    aStarPath[i].pos.y * nodeSize, 
                    nodeSize, nodeSize);
        }
    }
}

function aStarStart() {
    for (var row = 0; row < numNodesV; row++) {
        for (var col = 0; col < numNodesH; col++) {
            nodes[row][col].aStarInfo = new AStarNodeInfo();
        }
    }
    
    aStarOpenList[0] = startNode;
    aStarRunning = true;
}

function linearDist(n1, n2) {
    return Math.sqrt(Math.pow(n2.pos.x - n1.pos.x, 2) +  
                     Math.pow(n2.pos.y - n1.pos.y, 2));
}
function manhattanDist(n1, n2) {
    return Math.abs(n1.pos.x - n2.pos.y) + Math.abs(n1.pos.y - n2.pos.y);
}
function constDist(n1, n2) {
    return 5;
}
function aStarStep() {
    // Knoten mit kleinstem f-Wert extrahieren
    var smallest = Infinity;
    var cur = null;
    var index = 0;
    for (var i = 0; i < aStarOpenList.length; i++) {
        if(aStarOpenList[i].aStarInfo.f < smallest) {
            smallest = aStarOpenList[i].aStarInfo.f;
            index = i;
        }
    }
    var cur = aStarOpenList[index];
    aStarOpenList.splice(index, 1);

    // Ziel gefunden?
    if(cur == goalNode) {
        while(cur != null) {
            aStarPath.push(cur);
            cur = cur.aStarInfo.last;
        }
        aStarSucceeded = true;
        aStarRunning = false;
        return false;
    }

    // Knoten abschließen
    aStarClosedList.push(cur);

    // Nachbarn des Knotens in die Openlist schreiben
    for(var i = 0; i < cur.neighbours.length; i++) {
        var next = cur.neighbours[i];
        
        if(next.isWall == false)
        {
            // Wenn der Knoten schon in der Closedlist ist, nicht einfügen
            if(aStarClosedList.indexOf(next) != -1) {
                continue;
            }

            // g berechnen
            var g = cur.aStarInfo.g + 1; // Weg zwischen zwei Knoten = 1
            
            // Gucken, ob der Knoten schon auf der Openlist ist und ob sich der neue 
            // Weg lohnen würde
            var index = aStarOpenList.indexOf(next);
            if(index != -1 && g >= next.g) {
                continue;
            }

            // Vorgänger setzen
            next.aStarInfo.last = cur;
            next.aStarInfo.g = g;

            // f für den Knoten aktualisieren und (falls noch nicht getan) in die 
            // Openlist einfügen
            var f = g + linearDist(goalNode, next);
            if(index != -1) {
                next.aStarInfo.f = f;
            }
            else {
                next.aStarInfo.f = f;
                aStarOpenList.push(next);
            }
        }
    }
    
    // Wenn Openlist leer ist, beenden
    if(aStarOpenList.length == 0) {
        aStarRunning = false;
        return false;
    }
    
    return true;
}

function aStarReset() {
    aStarOpenList = [];
    aStarClosedList = [];
    aStarPath = [];
    aStarSucceeded = false;
    aStarRunning = false;
}
//////////////////////////////////////////////////////



//////////////////////////////////////////////////////
// DIJKSTRA STUFF
//
function DijkstraNodeInfo() {
    this.distance = Infinity;
    this.last = null;
}

var dijkstraOpenList = [];
var dijkstraClosedList = [];
var dijkstraSucceeded = false;
var dijkstraRunning = false;
var dijkstraPath = [];

function dijkstraRender() {
    dijkstraCtx.fillStyle = "#4499FF";
    for (var i = 0; i < dijkstraOpenList.length; i++) {
        dijkstraCtx.fillRect(dijkstraOpenList[i].pos.x * nodeSize, 
                dijkstraOpenList[i].pos.y * nodeSize, 
                nodeSize, nodeSize);
    }
    
    dijkstraCtx.fillStyle = "#0000FF";
    for (var i = 0; i < dijkstraClosedList.length; i++) {
        dijkstraCtx.fillRect(dijkstraClosedList[i].pos.x * nodeSize, 
                dijkstraClosedList[i].pos.y * nodeSize, 
                nodeSize, nodeSize);
    }
    
    if(dijkstraSucceeded == true) {
        dijkstraCtx.fillStyle = "#FFFF00";
        for (var i = 0; i < dijkstraPath.length; i++) {
            dijkstraCtx.fillRect(dijkstraPath[i].pos.x * nodeSize, 
                    dijkstraPath[i].pos.y * nodeSize, 
                    nodeSize, nodeSize);
        }
    }
}

function dijkstraStart() {
    for (var row = 0; row < numNodesV; row++) {
        for (var col = 0; col < numNodesH; col++) {
            nodes[row][col].dijkstraInfo = new DijkstraNodeInfo();
        }
    }
    
    dijkstraOpenList[0] = startNode;
    startNode.dijkstraInfo.dist = 0;
    dijkstraRunning = true;
}

function dijkstraStep() {
    var cur = dijkstraOpenList[0];
    dijkstraOpenList.shift();
    
    // Ziel gefunden?
    if(cur == goalNode) {
        while(cur != null) {
            dijkstraPath.push(cur);
            cur = cur.dijkstraInfo.last;
        }
        dijkstraSucceeded = true;
        dijkstraRunning = false;
        return false;
    }

    // Knoten abschließen
    dijkstraClosedList.push(cur);

    // Nachbarn des Knotens in die Openlist schreiben
    for(var i = 0; i < cur.neighbours.length; i++) {
        var next = cur.neighbours[i];
        
        if(next.isWall == false)
        {   
            // Wenn der Knoten schon in der Closedlist ist, nicht einfügen
            if(dijkstraClosedList.indexOf(next) != -1) {
                continue;
            }

            // Gucken, ob der Knoten schon auf der Openlist ist und ob sich der neue 
            // Weg lohnen würde
            var index = dijkstraOpenList.indexOf(next);
            
            // Weg berechnen
            var dist = cur.dijkstraInfo.dist + 1; // Weg zwischen zwei Knoten = 1
            
            if(index != -1) {
                if(dist < next.dist) {
                    next.dist = dist;
                    next.dijkstraInfo.last = cur;
                }
            } else {
                // in Openlist einfügen
                next.dijkstraInfo.last = cur;
                next.dijkstraInfo.dist = dist;
                dijkstraOpenList.push(next);
            }
        }
    }
    
    // Wenn Openlist leer ist, beenden
    if(dijkstraOpenList.length == 0) {
        dijkstraRunning = false;
        return false;
    }
    
    return true;
}

function dijkstraReset() {
    dijkstraOpenList = [];
    dijkstraClosedList = [];
    dijkstraPath = [];
    dijkstraSucceeded = false;
    dijkstraRunning = false;
}
//////////////////////////////////////////////////////



//////////////////////////////////////////////////////
// SHARED FUNCTIONS
//
function fillMap() {
    nodes = [];
    
    for (var row = 0; row < numNodesV; row++) {
        nodes[row] = [];
        
        for (var col = 0; col < numNodesH; col++) {
            nodes[row][col] = new Node(new Point(col, row));
        }
    }
    
    for (var row = 0; row < numNodesV; row++) {
        for (var col = 0; col < numNodesH; col++) {
            var node = nodes[row][col];
            node.neighbours = [];
            
            if(row + 1 < numNodesV) {
                node.neighbours.push(nodes[row + 1][col]);
            }
            if(row - 1 >= 0) {
                node.neighbours.push(nodes[row - 1][col]);
            }
            if(col + 1 < numNodesH) {
                node.neighbours.push(nodes[row][col + 1]);
            }
            if(col - 1 >= 0) {
                node.neighbours.push(nodes[row][col - 1]);
            }
        }
    }
    
    startNode = nodes[0][0];
    goalNode = nodes[4][0];
}

function render() {
    aStarCtx.fillStyle = "#FFFFFF";
    dijkstraCtx.fillStyle = "#FFFFFF";
    aStarCtx.fillRect(0, 0, width, height);
    dijkstraCtx.fillRect(0, 0, width, height);
    
    aStarCtx.fillStyle = "#000000";
    dijkstraCtx.fillStyle = "#000000";
    for (var row = 0; row < numNodesV; row++) {
        for (var col = 0; col < numNodesH; col++) {
            var node = nodes[row][col];
            
            if(node.isWall == true) {
                aStarCtx.fillRect(node.pos.x * nodeSize, 
                                node.pos.y * nodeSize, 
                                nodeSize, nodeSize);
                dijkstraCtx.fillRect(node.pos.x * nodeSize, 
                                node.pos.y * nodeSize, 
                                nodeSize, nodeSize);
            }
        }
    }
    
    aStarRender();
    dijkstraRender();
    
    if(startNode != null) {
        aStarCtx.fillStyle = "#00FF00";
        dijkstraCtx.fillStyle = "#00FF00";
        aStarCtx.fillRect(startNode.pos.x * nodeSize, 
                        startNode.pos.y * nodeSize, 
                        nodeSize, nodeSize);
        dijkstraCtx.fillRect(startNode.pos.x * nodeSize, 
                        startNode.pos.y * nodeSize, 
                        nodeSize, nodeSize);
    }
    
    if(goalNode != null) {
        aStarCtx.fillStyle = "#FF0000";
        dijkstraCtx.fillStyle = "#FF0000";
        aStarCtx.fillRect(goalNode.pos.x * nodeSize, 
                        goalNode.pos.y * nodeSize, 
                        nodeSize, nodeSize);
        dijkstraCtx.fillRect(goalNode.pos.x * nodeSize, 
                        goalNode.pos.y * nodeSize, 
                        nodeSize, nodeSize);
    }
}

function onClick(event) {
    var rect = event.target.getBoundingClientRect();
    var cursorPos = new Point(event.clientX - rect.left, event.clientY - rect.top);
    var row = parseInt(cursorPos.y / nodeSize);
    var col = parseInt(cursorPos.x / nodeSize);
    
    if(running == false) {
        var selected = fieldType.options[fieldType.selectedIndex].value;
        if(selected == "wall") {
            nodes[row][col].isWall = !nodes[row][col].isWall;
        } else if(selected == "start") {
            startNode = nodes[row][col];
        } else if(selected == "goal") {
            goalNode = nodes[row][col];
        }

        render();
    } else {
        if(event.target == aStarCanvas) {
            var f = nodes[row][col].aStarInfo.f;
            var g = nodes[row][col].aStarInfo.g;
            console.log("urg");
            window.alert("g= " + g + "\n" + "f= " + f + "\n");
        } else {
            
        }
    }
}

function start() {
    running = true;
    
    aStarStart();
    dijkstraStart();
    
    render();
    stepButton.disabled = false;
    finishButton.disabled = false;
    startButton.disabled = true;
    fieldType.disabled = true;
}

function reset() {
    running = false;
    
    aStarReset();
    dijkstraReset();
    
    render();
    stepButton.disabled = true;
    finishButton.disabled = true;
    startButton.disabled = false;
    fieldType.disabled = false;
}

function step() {
    if(aStarRunning == true) {
        aStarStep();
    }
    
    if(dijkstraRunning == true) {
        dijkstraStep();
    }
    
    render();
}

function finish() {
    if(aStarRunning == true) {
        while(aStarStep() == true);
    }
    
    if(dijkstraRunning == true) {
        while(dijkstraStep() == true);
    }
    
    render();
}
//////////////////////////////////////////////////////



//////////////////////////////////////////////////////
// MAIN
//
aStarCanvas.addEventListener("click", function (event) {onClick(event);});
dijkstraCanvas.addEventListener("click", function (event) {onClick(event);});

startButton.addEventListener("click", function (event) {start();});
resetButton.addEventListener("click", function (event) {reset();});
stepButton.addEventListener("click", function (event) {step();});
finishButton.addEventListener("click", function (event) {finish();});
stepButton.disabled = true;
finishButton.disabled = true;

fillMap();
render();
//////////////////////////////////////////////////////