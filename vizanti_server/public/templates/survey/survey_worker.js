let params = null;
let transect_labels = [];

const MAX_LINES = 1000;

//duplicated cause Firefox doesn't support worker module imports in all recent versions
function getCCWPolygon(poly){
	let area = 0;
	for (let i = 0; i < poly.length; i++) {
		const a = poly[i];
		const b = poly[(i+1) % poly.length];
		area += a.x * b.y - b.x * a.y;
	}

	if(area < 0)
		return poly.slice().reverse();
	return poly.slice();
}

function offsetPolygon(poly, dist){
	if(dist <= 0){
		return poly.map(p => ({
			x: p.x,
			y: p.y,
			z: p.z
		}));
	}

	// outward miter offset, assumes CCW winding so outward normal of edge (a->b) is (dy, -dx)
	const count = poly.length;
	const result = [];

	for(let i = 0; i < count; i++){
		const prev = poly[(i - 1 + count) % count];
		const cur = poly[i];
		const next = poly[(i + 1) % count];

		let dir0X = cur.x - prev.x;
		let dir0Y = cur.y - prev.y;
		let dir1X = next.x - cur.x;
		let dir1Y = next.y - cur.y;

		const len0 = Math.hypot(dir0X, dir0Y) || 1;
		const len1 = Math.hypot(dir1X, dir1Y) || 1;

		dir0X /= len0;
		dir0Y /= len0;
		dir1X /= len1;
		dir1Y /= len1;

		const norm0 = {
			x: dir0Y,
			y: -dir0X
		};

		const norm1 = {
			x: dir1Y,
			y: -dir1X
		};

		let miterX = norm0.x + norm1.x;
		let miterY = norm0.y + norm1.y;
		const miterLen = Math.hypot(miterX, miterY);

		if(miterLen < 1e-9){
			result.push({
				x: cur.x + norm0.x * dist,
				y: cur.y + norm0.y * dist,
				z: cur.z
			});
			continue;
		}

		miterX /= miterLen;
		miterY /= miterLen;

		const dot = norm0.x * norm1.x + norm0.y * norm1.y;
		let scale = dist / Math.sqrt((1 + dot) * 0.5);

		if(scale > dist * 3)
			scale = dist * 3;

		result.push({
			x: cur.x + miterX * scale,
			y: cur.y + miterY * scale,
			z: cur.z
		});
	}

	return result;
}

function closestOnPolygon(poly, point){
	let closest = null;

	for(let i = 0; i < poly.length; i++){
		const start = poly[i];
		const end = poly[(i + 1) % poly.length];

		const edgeX = end.x - start.x;
		const edgeY = end.y - start.y;
		const edgeLenSq = edgeX * edgeX + edgeY * edgeY;

		const relX = point.x - start.x;
		const relY = point.y - start.y;
		const dot = relX * edgeX + relY * edgeY;

		let t = edgeLenSq > 0 ? dot / edgeLenSq : 0;
		t = Math.max(0, Math.min(1, t));

		const closestX = start.x + t * edgeX;
		const closestY = start.y + t * edgeY;
		const dist = Math.hypot(point.x - closestX, point.y - closestY);

		if(closest == null || dist < closest.dist){
			closest = {
				edge: i,
				t,
				x: closestX,
				y: closestY,
				z: start.z + t * (end.z - start.z),
				dist
			};
		}
	}

	return closest;
}

function tracePerimeter(poly, from, to){
	// intermediate vertices between two boundary locations, along the shorter perimeter direction
	const n = poly.length;
	const edgeLen = [];
	let perimeter = 0;
	for (let i = 0; i < n; i++) {
		const a = poly[i];
		const b = poly[(i+1) % n];
		edgeLen.push(Math.hypot(b.x - a.x, b.y - a.y));
		perimeter += edgeLen[i];
	}
	if(perimeter < 1e-9)
		return [];

	function arclength(loc){
		let s = 0;
		for (let i = 0; i < loc.edge; i++)
			s += edgeLen[i];
		return s + edgeLen[loc.edge] * loc.t;
	}

	const sFrom = arclength(from);
	const sTo = arclength(to);
	const forward = (sTo - sFrom + perimeter) % perimeter;
	const backward = perimeter - forward;

	const verts = [];
	if(forward <= backward){
		let i = (from.edge + 1) % n;
		while(true){
			const sv = arclength({edge: i, t: 0});
			if(((sv - sFrom + perimeter) % perimeter) >= forward)
				break;
			verts.push({x: poly[i].x, y: poly[i].y, z: poly[i].z});
			i = (i + 1) % n;
			if(verts.length > n) break;
		}
	}else{
		let i = from.edge;
		while(true){
			const sv = arclength({edge: i, t: 0});
			if(((sFrom - sv + perimeter) % perimeter) >= backward)
				break;
			verts.push({x: poly[i].x, y: poly[i].y, z: poly[i].z});
			i = (i - 1 + n) % n;
			if(verts.length > n) break;
		}
	}
	return verts;
}

function pointInPolygon(poly, p){
	let inside = false;
	for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
		const a = poly[i];
		const b = poly[j];
		if(((a.y > p.y) != (b.y > p.y)) && (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)){
			inside = !inside;
		}
	}
	return inside;
}


function transectExtent(poly, angleRad){
	const nrm = {x: -Math.sin(angleRad), y: Math.cos(angleRad)};

	let min = Infinity, max = -Infinity;
	for (const p of poly) {
		const c = p.x * nrm.x + p.y * nrm.y;
		if(c < min) min = c;
		if(c > max) max = c;
	}

	const extent = max - min;
	return extent > 0 ? extent : 0;
}

function countTransects(poly, angleRad, spacing){
	const extent = transectExtent(poly, angleRad);
	if(extent == 0 || !(spacing > 0))
		return 0;

	const lines = Math.ceil((extent - spacing * 0.5) / spacing);
	return lines > 0 ? lines : 0;
}


function generateTransects(poly, angleRad, spacing, turnaround){

	function lineIntersections(poly, lineOrigin, lineDir) {
		// intersections of infinite line (p0 + t*dir) with polygon edges, z lerped along the edge
		const hits = [];

		for (let i = 0; i < poly.length; i++) {
			const edgeStart = poly[i];
			const edgeEnd = poly[(i + 1) % poly.length];

			const edgeDx = edgeEnd.x - edgeStart.x;
			const edgeDy = edgeEnd.y - edgeStart.y;

			const determinant = lineDir.x * edgeDy - lineDir.y * edgeDx;

			if (Math.abs(determinant) < 1e-12)
				continue;

			const edgeT = (lineDir.y * (edgeStart.x - lineOrigin.x) - lineDir.x * (edgeStart.y - lineOrigin.y)) / determinant;

			if (edgeT < 0 || edgeT >= 1)
				continue;

			const useX = Math.abs(lineDir.x) > Math.abs(lineDir.y);

			const intersectionCoord = useX ? edgeStart.x + edgeT * edgeDx : edgeStart.y + edgeT * edgeDy;
			const lineOriginCoord = useX ? lineOrigin.x : lineOrigin.y;
			const lineDirCoord = useX ? lineDir.x : lineDir.y;

			const lineT = (intersectionCoord - lineOriginCoord) / lineDirCoord;

			hits.push({
				t: lineT,
				x: edgeStart.x + edgeT * edgeDx,
				y: edgeStart.y + edgeT * edgeDy,
				z: edgeStart.z + edgeT * (edgeEnd.z - edgeStart.z)
			});
		}

		hits.sort((a, b) => a.t - b.t);
		return hits;
	}

	const dir = {x: Math.cos(angleRad), y: Math.sin(angleRad)};
	const nrm = {x: -dir.y, y: dir.x};

	let min = Infinity, max = -Infinity;
	for (const p of poly) {
		const c = p.x * nrm.x + p.y * nrm.y;
		if(c < min) min = c;
		if(c > max) max = c;
	}

	const segments = [];
	const lines = Math.ceil((max - min - spacing * 0.5) / spacing);

	for (let i = 0; i < lines; i++) {
		const c = min + spacing * (0.5 + i);
		const p0 = {x: nrm.x * c, y: nrm.y * c};
		let hits = lineIntersections(poly, p0, dir);

		// drop duplicate hits from lines passing exactly through a vertex shared by two edges
		hits = hits.filter((h, i) => i == 0 || h.t - hits[i-1].t > 1e-9);

		// keep only spans between consecutive hits whose midpoint lies inside the polygon, so concave shapes get one transect per interior span instead of garbage pairing
		for (let i = 0; i + 1 < hits.length; i++) {
			const a = hits[i];
			const b = hits[i+1];
			if(!pointInPolygon(poly, {x: (a.x + b.x) * 0.5, y: (a.y + b.y) * 0.5}))
				continue;
			segments.push({
				a: {x: a.x - dir.x * turnaround, y: a.y - dir.y * turnaround, z: a.z},
				b: {x: b.x + dir.x * turnaround, y: b.y + dir.y * turnaround, z: b.z}
			});
		}
	}
	return segments;
}

function makeCostCache(outer, needsRoute){

	const n = outer.length;
	const edgeStart = [];
	let perimeter = 0;

	for (let i = 0; i < n; i++) {
		const a = outer[i];
		const b = outer[(i+1) % n];
		edgeStart.push(perimeter);
		perimeter += Math.hypot(b.x - a.x, b.y - a.y);
	}

	function arclength(loc){
		const next = loc.edge + 1 < n ? edgeStart[loc.edge + 1] : perimeter;
		return edgeStart[loc.edge] + (next - edgeStart[loc.edge]) * loc.t;
	}

	function transitCost(p, q){
		const euclid = Math.hypot(q.x - p.x, q.y - p.y);
		if(params.direct_transit || !needsRoute(p, q))
			return euclid;

		const from = closestOnPolygon(outer, p);
		const to = closestOnPolygon(outer, q);

		if(perimeter < 1e-9)
			return from.dist + to.dist;

		const forward = (arclength(to) - arclength(from) + perimeter) % perimeter;
		return from.dist + Math.min(forward, perimeter - forward) + to.dist;
	}

	const ids = new WeakMap();
	let nextId = 0;
	const cache = new Map();

	return function(p, q){
		if(!ids.has(p))
			ids.set(p, nextId++);

		if(!ids.has(q))
			ids.set(q, nextId++);

		const i = ids.get(p), j = ids.get(q);
		const key = i < j ? i+","+j : j+","+i;

		let c = cache.get(key);
		if(c == undefined){
			c = transitCost(p, q);
			cache.set(key, c);
		}
		return c;
	};
}

function orderSegments(segments, entry, exit, cost){

	function twoOptImprove(order, entry, exit, cost){
		const n = order.length;

		function connect(p, q){
			return q == null ? 0 : cost(p, q);
		}

		let improved = true;
		while(improved){
			improved = false;
			for (let i = 0; i < n; i++) {
				for (let j = i; j < n; j++) {
					const prevEnd = i == 0 ? entry : order[i-1].b;
					const nextStart = j == n-1 ? exit : order[j+1].a;
					const before = cost(prevEnd, order[i].a) + connect(order[j].b, nextStart);
					const after = cost(prevEnd, order[j].b) + connect(order[i].a, nextStart);
					if(after < before - 1e-9){
						const block = order.slice(i, j+1).reverse().map(s => ({a: s.b, b: s.a}));
						order.splice(i, j - i + 1, ...block);
						improved = true;
					}
				}
			}
		}
	}

	if(segments.length == 0)
		return [];

	// greedy nearest-neighbor construction over all segments by true transit cost
	const remaining = segments.slice();
	const order = [];

	let cur = entry;
	while(remaining.length > 0){
		let bestIdx = 0
		let bestFlip = false
		let bestCost = Infinity;

		for (let i = 0; i < remaining.length; i++) {

			const cA = cost(cur, remaining[i].a);
			const cB = cost(cur, remaining[i].b);

			if(cA < bestCost){
				bestCost = cA;
				bestIdx = i;
				bestFlip = false;
			}

			if(cB < bestCost){
				bestCost = cB;
				bestIdx = i;
				bestFlip = true;
			}
		}

		const seg = remaining.splice(bestIdx, 1)[0];
		if (bestFlip){
			order.push({
				a: seg.b,
				b: seg.a
			});
		}else{
			order.push({
				a: seg.a,
				b: seg.b
			});
		}
		cur = order[order.length-1].b;
	}

	twoOptImprove(order, entry, exit, cost);
	return order;
}

function sampleStep(poly, spacing, tolerance){

	let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
	for (const p of poly) {
		if(p.x < minX)
			minX = p.x;

		if(p.y < minY)
			minY = p.y;

		if(p.x > maxX)
			maxX = p.x;

		if(p.y > maxY)
			maxY = p.y;
	}

	const diag = Math.hypot(maxX - minX, maxY - minY);
	return Math.max(tolerance, Math.min(spacing, diag / 200) * 0.5);
}

function makeConnectorCheck(poly, turnaround, spacing, tolerance){

	const step = sampleStep(poly, spacing, tolerance);
	const band = turnaround + tolerance;

	return function(p1, p2){
		const len = Math.hypot(p2.x - p1.x, p2.y - p1.y);
		const n = Math.max(2, Math.ceil(len / step));
		for (let i = 1; i < n; i++) {
			const t = i / n;
			const s = {
				x: p1.x + (p2.x - p1.x) * t,
				y: p1.y + (p2.y - p1.y) * t
			};

			if(closestOnPolygon(poly, s).dist > band)
				return true;
		}
		return false;
	};
}

function makeInteriorCheck(poly, spacing, tolerance){
	// approach legs are free to travel outside the polygon, they only may not cut through the survey interior
	const step = sampleStep(poly, spacing, tolerance);

	return function(p1, p2){
		const len = Math.hypot(p2.x - p1.x, p2.y - p1.y);
		const n = Math.max(2, Math.ceil(len / step));
		for (let i = 1; i < n; i++) {
			const t = i / n;
			const s = {
				x: p1.x + (p2.x - p1.x) * t,
				y: p1.y + (p2.y - p1.y) * t
			};

			if(pointInPolygon(poly, s) && closestOnPolygon(poly, s).dist > tolerance)
				return true;
		}
		return false;
	};
}

function shortcutChain(chain, blocked){
	const result = [chain[0]];
	let i = 0;

	while(i < chain.length - 1){
		let next = i + 1;
		for (let j = chain.length - 1; j > i + 1; j--) {
			if(!blocked(chain[i], chain[j])){
				next = j;
				break;
			}
		}
		result.push(chain[next]);
		i = next;
	}

	return result;
}

function pushApproach(path, cur, target, outer, blocked){
	const chain = [cur, closestOnPolygon(outer, cur)];
	const to = closestOnPolygon(outer, target);

	for (const v of tracePerimeter(outer, chain[1], to))
		chain.push(v);

	chain.push(to);
	chain.push(target);

	for (const p of shortcutChain(chain, blocked).slice(1))
		pushUnique(path, {x: p.x, y: p.y, z: p.z, transit: true});
}

function appendTransects(path, segs, outer, needsRoute, approach){
	for (const seg of segs) {
		const cur = path[path.length-1];

		// path holds only the start marker on the first pass, so this is the
		// approach leg. the crosshatch handoff keeps the band rule
		if(cur && approach != null && path.length == 1){
			pushApproach(path, cur, seg.a, outer, approach);
			pushUnique(path, seg.b);
			transect_labels.push({x: (seg.a.x + seg.b.x) * 0.5, y: (seg.a.y + seg.b.y) * 0.5});
			continue;
		}

		if(cur && needsRoute(cur, seg.a)){
			if(params.direct_transit){
				pushUnique(path, {x: seg.a.x, y: seg.a.y, z: seg.a.z, transit: true});
			}else{
				// route along the turnaround boundary instead of crossing uncharted
				// area or cutting through the survey interior
				const from = closestOnPolygon(outer, cur);
				const to = closestOnPolygon(outer, seg.a);
				pushUnique(path, from);
				for (const v of tracePerimeter(outer, from, to))
					pushUnique(path, v);
				pushUnique(path, to);
				pushUnique(path, seg.a);
			}
		}else{
			pushUnique(path, seg.a);
		}
		pushUnique(path, seg.b);
		transect_labels.push({x: (seg.a.x + seg.b.x) * 0.5, y: (seg.a.y + seg.b.y) * 0.5});
	}
}

function pushUnique(list, p){
	const last = list[list.length-1];
	if(last && Math.hypot(last.x - p.x, last.y - p.y) < 1e-6)
		return;
	list.push({x: p.x, y: p.y, z: p.z, transit: p.transit === true});
}


function generateSurvey(){

	const polygon = params.polygon;
	const start_marker = params.start_marker;
	const end_marker = params.end_marker;

	if(polygon.length < 3 || !start_marker || !end_marker)
		return {survey_points: [], transect_labels: [], error: null};

	const spacing = Math.max(params.spacing, 0.05);
	const angle = params.angle;
	const turnaround = Math.max(params.turnaround, 0);

	transect_labels = [];

	const poly = getCCWPolygon(polygon);

	let total = countTransects(poly, angle, spacing);
	if(params.crosshatch)
		total += countTransects(poly, angle + Math.PI/2, spacing);

	if(total > MAX_LINES)
		return {survey_points: [], transect_labels: [], error: "Survey needs "+total+" lines, over the limit of "+MAX_LINES+". Increase the line spacing or shrink the area."};

	const outer = offsetPolygon(poly, turnaround);
	const tolerance = Math.max(0.01, turnaround * 0.05);
	const needsRoute = makeConnectorCheck(poly, turnaround, spacing, tolerance);
	const cost = makeCostCache(outer, needsRoute);

	const crossesInterior = makeInteriorCheck(poly, spacing, tolerance);
	const startApproach = params.direct_transit || pointInPolygon(poly, start_marker) ? null : crossesInterior;
	const endApproach = params.direct_transit || pointInPolygon(poly, end_marker) ? null : crossesInterior;

	const mainSegments = generateTransects(poly, angle, spacing, turnaround);
	if(mainSegments.length == 0)
		return {survey_points: [], transect_labels: [], error: null};

	const path = [];
	pushUnique(path, start_marker);

	if(params.crosshatch){
		const pass = orderSegments(mainSegments, start_marker, null, cost);
		appendTransects(path, pass, outer, needsRoute, startApproach);

		const crossSegments = generateTransects(poly, angle + Math.PI/2, spacing, turnaround);
		const cross = orderSegments(crossSegments, path[path.length-1], end_marker, cost);
		appendTransects(path, cross, outer, needsRoute, null);
	}else{
		const pass = orderSegments(mainSegments, start_marker, end_marker, cost);
		appendTransects(path, pass, outer, needsRoute, startApproach);
	}

	// connect to the end marker, directly when allowed, along the boundary otherwise
	const last = path[path.length-1];

	if(endApproach != null){
		pushApproach(path, last, end_marker, outer, endApproach);
	}else{
		if(!params.direct_transit && needsRoute(last, end_marker)){
			const exit = closestOnPolygon(outer, last);
			const depart = closestOnPolygon(outer, end_marker);
			pushUnique(path, exit);
			for (const v of tracePerimeter(outer, exit, depart))
				pushUnique(path, v);
			pushUnique(path, depart);
		}
		pushUnique(path, end_marker);
	}

	return {survey_points: path, transect_labels: transect_labels, error: null};
}

self.addEventListener('message', function(event) {
	params = event.data;

	let result;
	try {
		result = generateSurvey();
	} catch (error) {
		result = {
            survey_points: [], 
            transect_labels: [], 
            error: "Path generation failed: "+error.message
        };
	}

	result.seq = params.seq;
	self.postMessage(result);
}, false);