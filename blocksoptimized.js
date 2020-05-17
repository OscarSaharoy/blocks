// Oscar Saharoy 2020

(function () {

    // define vectors and vector math
    class Vector {
        constructor(x, y, z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        mod() {
            return Math.sqrt(this.x*this.x + this.y*this.y + this.z*this.z);
        }

        arg() {
            return Math.atan2(this.y, this.x);
        }
    }

    function add(a, b) {
        return new Vector(a.x+b.x, a.y+b.y, a.z+b.z);
    }

    function subtract(a, b) {
        return new Vector(a.x-b.x, a.y-b.y, a.z-b.z);
    }

    function scale(a, s) {
        return new Vector(a.x*s, a.y*s, a.z*s);
    }

    function dot(a, b) {
        return a.x*b.x + a.y*b.y + a.z*b.z;
    }

    function cross(a, b) {
        return new Vector(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
    }

    function rotate(a, th) {
        return new Vector(a.x*Math.cos(th) - a.y*Math.sin(th), a.x*Math.sin(th) + a.y*Math.cos(th), a.z);
    }

    // define unit and zero vectors
    var unitIVector = new Vector(1, 0, 0);
    var unitJVector = new Vector(0, 1, 0);
    var unitKVector = new Vector(0, 0, 1);
    var zeroVector  = new Vector(0, 0, 0);

    // line class basically 2 Vectors
    class Line {
        constructor(a, t) {
            this.a = a; // offset vector
            this.t = t; // direction vector
        }

        // point on that line
        r(mu) {
            return add(this.a, scale(this.t, mu));
        }

        // rotate line
        rotate(th) {
            this.a = rotate(this.a, th);
            this.t = rotate(this.t, th);
        }
    }

    // check if lines intersect
    function checkIntersection(l1, l2) {
        
        // find mu parameters at point of intersection
        var mu1 = cross(subtract(l1.a, l2.a), l2.t).z / cross(l1.t, l2.t).z;
        var mu2 = cross(subtract(l1.a, l2.a), l1.t).z / cross(l1.t, l2.t).z;

        // if intersection point is within the first t vector of both lines, they intersect
        return mu1 >= 0 && mu1 <= 1 && mu2 >= 0 && mu2 <= 1;
    }

    function lineNormal(p, l) {

        var mu = dot(subtract(p, l.a), l.t) / dot(l.t, l.t);

        if(mu <= 0 || mu >= 1) {
            return false;
        }

        var n = subtract(p, l.r(mu));
        return scale(n, 1/n.mod());
    }

    // vertex class to store vectices of blocks
    class Vertex {
        constructor(point, line, n) {
            this.point   = point;
            this.line    = line;
            this.n       = n;

            this.normalsIndices = [];
            this.normals        = [];
        }
    }

    // block class is a polygon in space
    class Block {
        constructor(index, com, vel, w, points, mass=null) {
            this.com      = com;   // centre of mass position
            this.segment  = 0;     // which screen segment the block is in
            this.index    = index; // index of this block in blocks array
            this.vel      = vel;   // velocity
            this.w        = w;     // angular velocity
            this.rest     = 1.0;   // restitution
            this.friction = 0.0;   // coefficient friction

            this.area     = 0;
            this.J        = mass || 0;
            var loop      = points.concat(points.slice(0,1));
            var cx        = 0;
            var cy        = 0;

            this.vertices = [];

            // loop over points to calculate properties of polygon
            for(var i=0; i<loop.length-1; ++i) {

                // current and next point
                var point  = loop[i];
                var next   = loop[i+1];

                this.area -= point.x * next.y - next.x * point.y; // shoelace formula
                cx        -= (point.x + next.x) * (point.x * next.y - next.x * point.y);
                cy        -= (point.y + next.y) * (point.x * next.y - next.x * point.y);
                this.J    -= (point.y*point.y + point.y*next.y + next.y*next.y + point.x*point.x + point.x*next.x + next.x*next.x) * (point.x * next.y - next.x * point.y);
            
                // create line from one point to next
                var t      = subtract(next, point);
                var line   = new Line(point, t);

                // unit normal to line facing exterior of block
                var n      = scale(cross(unitKVector, t), 1/t.mod());

                // add vertex
                this.vertices.push(new Vertex(point, line, n));
            }

            // set area, mass, centroid and moment of inertia
            this.area /= 2;
            this.m     = mass || this.area; // set mass to equal area if mass argument has not been passed
            cx        /= (6 * this.area);
            cy        /= (6 * this.area);
            this.J    /= 12 * 0.1;

            // shift points vectors to be relative to actual centre of mass
            for(var vertex of this.vertices) {
                vertex.point.x -= cx;
                vertex.point.y -= cy;
            }
        }

        draw(lines=false, normals=false) {

            // start path with black as fill style
            ctx.fillStyle = '#000';
            ctx.beginPath();

            // for each vertex move to its position on screen, drawing a line if it's not the first vertex
            for(var vertex of this.vertices) {
                var absPoint = add(this.com, vertex.point);
                var absLine  = add(absPoint, vertex.line.t);
                normals ? ctx.moveTo(absPoint.x, absPoint.y) : 0;
                ctx.lineTo(absLine.x, absLine.y);

                // if normals is true then draw line normals
                if(normals) {
                    var mid = add(this.com, vertex.line.r(0.5));
                    var midPlusN = add(mid, scale(vertex.n, 100));
                    ctx.moveTo(mid.x, mid.y);
                    ctx.lineTo(midPlusN.x, midPlusN.y);
                }
            }

            // close up path and fill or stroke if lines is true
            ctx.closePath();
            lines ? ctx.stroke() : ctx.fill();
        }

        step(segments) {

            // if mass is infinite (wall blocks) then skip
            if(this.m == Infinity) {
                return;
            }

            // effect of angular velocity - rotate all vertices around centre of mass
            for(var vertex of this.vertices) {
                vertex.point = rotate(vertex.point, this.w.z);
                vertex.n     = rotate(vertex.n, this.w.z);
                vertex.line.rotate(this.w.z);
            }

            // add gravity
            this.vel.y += g;

            // increment position of centre of mass by velocity
            this.com.y += this.vel.y - 1/2 * g; // taylor series correction
            this.com.x += this.vel.x;

            // get which segment its in and add to segments array
            var xSegment = Math.floor(this.com.x/segmentLength);
            var ySegment = Math.floor(this.com.y/segmentLength);

            this.segment = xSegment + ySegment * xSegments;
            (segments[this.segment] || []).push(this.index);
        }

        collisionCheck(segments) {

            // compile indices of blocks to check
            var checkIndices = walls.concat(segments[this.segment], segments[this.segment-1], segments[this.segment+1], segments[this.segment+xSegments], segments[this.segment-xSegments],
                                            segments[this.segment-xSegments-1], segments[this.segment-xSegments+1], segments[this.segment+xSegments-1], segments[this.segment+xSegments+1]);

            // if a block has left area around current block then remove its normals
            for(var vertex1 of this.vertices) {

                for(var i=0; i<vertex1.normalsIndices.length; ++i) {

                    var index = vertex1.normalsIndices[i];

                    if(!checkIndices.includes(index)) {
                        vertex1.normalsIndices.splice(i,1);
                        vertex1.normals.splice(i,1);
                        i--;
                    }
                }
            }

            // loop over blocks
            for(var i of checkIndices) {

                if(i == undefined) {
                    continue;
                }

                var block2 = blocks[i];

                // don't collide block with itself
                if(this == block2) {
                    continue;
                }

                for(var vertex1 of this.vertices) {

                    // store for dot products of current normal vectors with each line normal
                    var normals = [];

                    // get index of normals to block2 in vertex1.normals
                    var index = vertex1.normalsIndices.indexOf(block2.index);
                    var j=0;

                    for(var vertex2 of block2.vertices) {

                        // get normal vector at current position (nc)
                        var pc  = add(vertex1.point, this.com);
                        var lc  = new Line(add(vertex2.line.a, block2.com), vertex2.line.t);
                        var nc  = lineNormal(pc, lc);

                        // dot product of current normal with line normal (==1 or -1)
                        var ncd = dot(nc, vertex2.n);

                        // continue only if previous normal exists
                        if(index != -1) {

                            // get previous normal vector dot with line normal
                            var npd = vertex1.normals[index][j];
                            ++j;

                            // collision occured if ncd is negative and npd is positive
                            if(ncd < 0 && npd > 0) {

                                // collide
                                this.collide(block2, vertex1, vertex2.n);
                            }
                        }

                        // store normal of this line in normals
                        normals.push(ncd);
                    }

                    // if vertex1.normals doesnt contain the normals for this block, add them
                    if(index == -1) {

                        // store current normals in vertex object 
                        vertex1.normalsIndices.push(block2.index);
                        vertex1.normals.push(normals);
                    }
                    else {

                        // store current normals in vertex object
                        vertex1.normals[index] = normals;
                    }
                }
            }
        }

        collide(block2, vertex1, n) {

            // calculate velocities after collision
            var r1     = vertex1.point;
            var r2     = subtract(add(r1, this.com), block2.com);

            var vr     = subtract(add(this.vel, cross(this.w, r1)), add(block2.vel, cross(block2.w, r2)));
            var t      = subtract(vr, scale(n, dot(vr, n)));
            t          = scale(t, 1/t.mod());

            var I_n    = -(1+this.rest)*dot(subtract(add(this.vel, cross(this.w, r1)), add(block2.vel, cross(block2.w, r2))), n);
            I_n       /= (1/this.m + 1/block2.m + 1/this.J * (dot(r1, r1) - dot(r1, n)*dot(r1, n)) + 1/block2.J * (dot(r2, r2) - dot(r2, n)*dot(r2, n)) );
            
            if(I_n == NaN) {
                return;
            }

            var I_t    = -this.friction*dot(subtract(add(this.vel, cross(this.w, r1)), add(block2.vel, cross(block2.w, r2))), t);
            I_t       /= (1/this.m + 1/block2.m + 1/this.J * (dot(r1, r1) - dot(r1, t)*dot(r1, t)) + 1/block2.J * (dot(r2, r2) - dot(r2, t)*dot(r2, t)) );
            
            var I      = add(scale(n, I_n), scale(t, I_t));

            var dv1    = scale(I, 1/this.m);
            var dv2    = scale(I, -1/block2.m);
            var dw1    = scale(cross(r1, I), 1/this.J);
            var dw2    = scale(cross(r2, I), -1/block2.J);

            this.vel   = add(this.vel, dv1);
            this.w     = add(this.w, dw1);
            block2.vel = add(block2.vel, dv2);
            block2.w   = add(block2.w, dw2);
        }
    }

    function resize() {

        dpr = window.devicePixelRatio || 1; // get dpr
        w   = parseInt(dpr * Math.max(document.documentElement.clientWidth,  window.innerWidth)); // get screen width of canvas in px
        h   = parseInt(dpr * Math.max(document.documentElement.clientHeight, window.innerHeight)); // get screen height of canvas in px

        // set canvas width to its screen width (max width = 2000px)
        c.width  = w;
        c.height = h;

        // set number of segments in each direction
        xSegments = Math.ceil(w/segmentLength);
        ySegments = Math.ceil(h/segmentLength);
    }

    function makeBlocks(N) {

        // four blocks to make walls
        var blocks = [new Block(0, new Vector( -100, h/2, 0), zeroVector, zeroVector, 
                        [new Vector(-100, -h/2, 0),
                         new Vector(-100,  h/2, 0),
                         new Vector( 100,  h/2, 0),
                         new Vector( 100, -h/2, 0)], mass=Infinity),
                      new Block(1, new Vector(w+100, h/2, 0), zeroVector, zeroVector, 
                        [new Vector(-100, -h/2, 0),
                         new Vector(-100,  h/2, 0),
                         new Vector( 100,  h/2, 0),
                         new Vector( 100, -h/2, 0)], mass=Infinity),
                      new Block(2, new Vector(w/2,  -100, 0), zeroVector, zeroVector,
                        [new Vector(-w/2, -100, 0),
                         new Vector(-w/2,  100, 0),
                         new Vector( w/2,  100, 0),
                         new Vector( w/2, -100, 0)], mass=Infinity),
                      new Block(3, new Vector(w/2, h+100, 0), zeroVector, zeroVector,
                        [new Vector(-w/2, -100, 0),
                         new Vector(-w/2,  100, 0),
                         new Vector( w/2,  100, 0),
                         new Vector( w/2, -100, 0)], mass=Infinity)];

        var points = [new Vector(-10,-10,0),
                      new Vector(-10,10,0),
                      new Vector(10,10,0),
                      new Vector(10,-10,0)];

        for(var i=4; i<N+4; ++i) {

            var com = new Vector(Math.random() * w, Math.random() * h, 0);
            var vel = new Vector(Math.random()*10, Math.random()*10, 0);
            var thd = new Vector(0, 0, Math.random()*0.01);
            blocks.push(new Block(i, com, vel, thd, points.slice()));
        }

        return blocks;
    }

    function segmentScreen() {

        // canvas segementation
        var segments = Array.from(Array(xSegments * ySegments), () => []);

        return segments;
    }

    function drawFrame() {

        // clear screen
        ctx.clearRect(0, 0, w, h);

        // reset segments array
        var segments = segmentScreen();

        // move blocks by 1 step
        for(var block of blocks) {

            block.step(segments);
        }

        // calculate collisions
        for(var block of blocks) {

            block.collisionCheck(segments);
        }

        // draw blocks
        for(var block of blocks) {
            block.draw(lines=false, normals=false);
        }

        // repeat
        requestAnimationFrame(drawFrame);
    }

    // canvas element
    var c    = document.getElementById("blocks");
    var ctx  = c.getContext("2d");

    var xSegments, ySegments;
    var segmentLength = 20;

    // handle window sizing
    resize();
    window.addEventListener("resize", resize);

    // make blocks
    var blocks = makeBlocks(300);
    var g = 0.002; // gravitational field strength

    // indices of walls in blocks array
    var walls = [0,1,2,3];

    // start simulation
    drawFrame();

})();