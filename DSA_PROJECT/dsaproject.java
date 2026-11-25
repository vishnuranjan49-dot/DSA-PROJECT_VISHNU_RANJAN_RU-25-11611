import java.util.*;

/*
 * Traffic Signal Simulator
 * Uses: Circular Queue, Graph, Priority Queue, Greedy algorithm
 * Compile : javac TrafficSignalSimulator.java
 * Run     : java TrafficSignalSimulator
 */

// ---------------------------------------------------------------- Vehicle (data model)
class Vehicle {
    private String id;
    private char lane;
    private int arrivalTime;
    private int priority;      // 1 Ambulance, 2 Fire Truck, 3 Bus, 4 Car
    private int passTime;
    private int waitingTime;

    public Vehicle(String id, char lane, int arrivalTime, int priority) {
        this.id = id;
        this.lane = lane;
        this.arrivalTime = arrivalTime;
        this.priority = priority;
    }

    public String getId() { return id; }
    public char getLane() { return lane; }
    public int getArrivalTime() { return arrivalTime; }
    public int getPriority() { return priority; }
    public int getPassTime() { return passTime; }
    public int getWaitingTime() { return waitingTime; }

    public void setPassTime(int t) {
        passTime = t;
        waitingTime = t - arrivalTime;
    }

    public Vehicle copy() { return new Vehicle(id, lane, arrivalTime, priority); }

    public String toString() {
        return id + " lane=" + lane + " AT=" + arrivalTime + " Pr=" + priority;
    }
}

// ---------------------------------------------------------------- Circular Queue
class CircularQueue {
    private Vehicle[] arr;
    private int front, rear, count, capacity;

    public CircularQueue(int capacity) {
        this.capacity = capacity;
        arr = new Vehicle[capacity];
        front = 0;
        rear = -1;
        count = 0;
    }

    public boolean enqueue(Vehicle v) {
        if (isFull()) return false;
        rear = (rear + 1) % capacity;      // wrap-around
        arr[rear] = v;
        count++;
        return true;
    }

    public Vehicle dequeue() {
        if (isEmpty()) return null;
        Vehicle v = arr[front];
        arr[front] = null;
        front = (front + 1) % capacity;
        count--;
        return v;
    }

    public Vehicle peek() { return isEmpty() ? null : arr[front]; }

    // i-th vehicle from the front (0 = front)
    public Vehicle get(int i) { return arr[(front + i) % capacity]; }

    public boolean isEmpty() { return count == 0; }
    public boolean isFull() { return count == capacity; }
    public int size() { return count; }
}

// ---------------------------------------------------------------- Lane
class Lane {
    private char name;
    private CircularQueue vehicles;
    private int passed;

    public Lane(char name) {
        this.name = name;
        vehicles = new CircularQueue(200);
    }

    public char getName() { return name; }
    public boolean addVehicle(Vehicle v) { return vehicles.enqueue(v); }
    public int size() { return vehicles.size(); }
    public int getPassed() { return passed; }

    // number of vehicles at the front of the queue that have already arrived at time t
    public int ready(int t) {
        int r = 0;
        while (r < vehicles.size() && vehicles.get(r).getArrivalTime() <= t) r++;
        return r;
    }

    // most urgent (smallest) priority among the ready vehicles, 99 if none
    public int bestPriority(int t) {
        int best = 99;
        for (int i = 0; i < ready(t); i++) best = Math.min(best, vehicles.get(i).getPriority());
        return best;
    }

    // position (1-based) of the first ready vehicle having priority p
    public int positionOf(int p, int t) {
        for (int i = 0; i < ready(t); i++)
            if (vehicles.get(i).getPriority() == p) return i + 1;
        return 1;
    }

    // earliest arrival time of the vehicle at the front, -1 if lane is empty
    public int nextArrival() { return vehicles.isEmpty() ? -1 : vehicles.peek().getArrivalTime(); }

    public Vehicle release(int passTime) {
        Vehicle v = vehicles.dequeue();
        v.setPassTime(passTime);
        passed++;
        return v;
    }

    public int totalVehicles() { return vehicles.size(); }

    public List<Vehicle> snapshot() {
        List<Vehicle> list = new ArrayList<>();
        for (int i = 0; i < vehicles.size(); i++) list.add(vehicles.get(i));
        return list;
    }
}

// ---------------------------------------------------------------- Graph
class RoadGraph {
    static class Edge {
        String to;
        int time;
        Edge(String to, int time) { this.to = to; this.time = time; }
    }

    private Map<String, List<Edge>> adj = new TreeMap<>();

    public void addIntersection(String name) { adj.putIfAbsent(name, new ArrayList<>()); }

    public void addRoad(String a, String b, int time) {
        addIntersection(a);
        addIntersection(b);
        adj.get(a).add(new Edge(b, time));
        adj.get(b).add(new Edge(a, time));       // undirected road
    }

    public List<Edge> getNeighbors(String v) { return adj.getOrDefault(v, new ArrayList<>()); }
    public boolean has(String v) { return adj.containsKey(v); }
    public int size() { return adj.size(); }

    public void display() {
        if (adj.isEmpty()) { System.out.println("Road map is empty."); return; }
        System.out.println("Road Map (adjacency list):");
        for (Map.Entry<String, List<Edge>> en : adj.entrySet()) {
            StringBuilder sb = new StringBuilder(en.getKey() + " ->");
            for (Edge e : en.getValue()) sb.append(" ").append(e.to).append("(").append(e.time).append(")");
            System.out.println(sb);
        }
    }
}

// ---------------------------------------------------------------- Greedy path search (Dijkstra)
class PathFinder {
    public static int lastCost;

    public static List<String> shortestPath(RoadGraph g, String src, String dst) {
        Map<String, Integer> dist = new HashMap<>();
        Map<String, String> prev = new HashMap<>();
        Set<String> done = new HashSet<>();
        PriorityQueue<Object[]> pq = new PriorityQueue<>((x, y) -> (Integer) x[0] - (Integer) y[0]);
        dist.put(src, 0);
        pq.add(new Object[]{0, src});
        while (!pq.isEmpty()) {
            Object[] top = pq.poll();                    // greedy: nearest intersection first
            String u = (String) top[1];
            if (!done.add(u)) continue;
            if (u.equals(dst)) break;
            for (RoadGraph.Edge e : g.getNeighbors(u)) relax(u, e, dist, prev, pq);
        }
        if (!dist.containsKey(dst)) return null;
        LinkedList<String> path = new LinkedList<>();
        for (String at = dst; at != null; at = prev.get(at)) path.addFirst(at);
        lastCost = dist.get(dst);
        return path;
    }

    private static void relax(String u, RoadGraph.Edge e, Map<String, Integer> dist,
                              Map<String, String> prev, PriorityQueue<Object[]> pq) {
        int nd = dist.get(u) + e.time;
        if (nd < dist.getOrDefault(e.to, Integer.MAX_VALUE)) {
            dist.put(e.to, nd);
            prev.put(e.to, u);
            pq.add(new Object[]{nd, e.to});
        }
    }
}

// ---------------------------------------------------------------- Abstract controller
abstract class SignalController {
    protected String intersectionName;
    protected Lane[] lanes = new Lane[4];
    protected ArrayList<String> timelineLanes = new ArrayList<>();
    protected ArrayList<Integer> timelineTimes = new ArrayList<>();
    protected List<Vehicle> passedVehicles = new ArrayList<>();
    protected int time = 0;
    protected int idleGreen = 0;
    protected int maxGreen = 6;

    public SignalController(String name, Lane[] source) {
        intersectionName = name;
        for (int i = 0; i < 4; i++) {                   // work on a private copy of the traffic
            lanes[i] = new Lane(source[i].getName());
            for (Vehicle v : source[i].snapshot()) lanes[i].addVehicle(v.copy());
        }
    }

    public abstract void runControl();
    public abstract String modeName();

    protected boolean vehiclesLeft() {
        for (Lane l : lanes) if (l.size() > 0) return true;
        return false;
    }

    // give green to a lane for `units` time units (one vehicle crosses per unit)
    protected void serve(Lane lane, int units) {
        int start = time;
        for (int u = 0; u < units; u++) {
            if (lane.ready(time) > 0) {
                time++;
                passedVehicles.add(lane.release(time));
            } else {
                time++;
                idleGreen++;
            }
        }
        if (timelineLanes.isEmpty() || time > start) {
            timelineLanes.add(String.valueOf(lane.getName()));
            timelineTimes.add(start);
        }
    }

    // jump the clock to the next arrival when nothing is waiting
    protected void waitForArrival() {
        int next = Integer.MAX_VALUE;
        for (Lane l : lanes) if (l.nextArrival() >= 0) next = Math.min(next, l.nextArrival());
        if (next > time) time = next;
    }

    protected Lane greedyLane() {
        PriorityQueue<Lane> heap = new PriorityQueue<>((a, b) -> b.ready(time) - a.ready(time));
        for (Lane l : lanes) if (l.ready(time) > 0) heap.add(l);   // max-heap on waiting vehicles
        return heap.poll();
    }

    public double getAvgWaitingTime() {
        if (passedVehicles.isEmpty()) return 0;
        int s = 0;
        for (Vehicle v : passedVehicles) s += v.getWaitingTime();
        return (double) s / passedVehicles.size();
    }

    public int getTotalIdleGreen() { return idleGreen; }
    public int getClearanceTime() {
        int m = 0;
        for (Vehicle v : passedVehicles) m = Math.max(m, v.getPassTime());
        return m;
    }

    // waiting time of the most urgent vehicle (priority 1), -1 if none
    public int getAmbulanceWait() {
        for (Vehicle v : passedVehicles) if (v.getPriority() == 1) return v.getWaitingTime();
        return -1;
    }

    public void printTimeline() {
        System.out.println("Signal Timeline:");
        StringBuilder seg = new StringBuilder();
        StringBuilder tm = new StringBuilder();
        for (int i = 0; i < timelineLanes.size(); i++) {
            seg.append("| ").append(timelineLanes.get(i)).append("  ");
            tm.append(String.format("%-5d", timelineTimes.get(i)));
        }
        seg.append("|");
        tm.append(timelineTimes.get(timelineTimes.size() - 1) + lastGreen());
        System.out.println(seg);
        System.out.println(tm);
    }

    private int lastGreen() { return time - timelineTimes.get(timelineTimes.size() - 1); }

    public void printResults() {
        System.out.println(String.format("%-6s%-5s%-4s%-4s%-5s%-7s%s", "LANE", "VEH", "AT", "Pr", "CT", "AWT", "MWT"));
        System.out.println("----------------------------------");
        for (Lane l : lanes) {
            int n = 0, sum = 0, mx = 0, ct = 0, pr = 99;
            int at = Integer.MAX_VALUE;
            for (Vehicle v : passedVehicles) {
                if (v.getLane() != l.getName()) continue;
                n++;
                sum += v.getWaitingTime();
                mx = Math.max(mx, v.getWaitingTime());
                ct = Math.max(ct, v.getPassTime());
                pr = Math.min(pr, v.getPriority());
                at = Math.min(at, v.getArrivalTime());
            }
            if (n == 0) { System.out.println(String.format("%-6s%-5d%-4s%-4s%-5s%-7s%s", l.getName(), 0, "-", "-", "-", "-", "-")); continue; }
            System.out.println(String.format("%-6s%-5d%-4d%-4d%-5d%-7.2f%d", l.getName(), n, at, pr, ct, (double) sum / n, mx));
        }
        System.out.println();
        System.out.println(String.format("Avg WT = %.2f | Idle Green = %d", getAvgWaitingTime(), idleGreen));
    }
}

// ---------------------------------------------------------------- Fixed-Time (circular rotation)
class FixedTimeController extends SignalController {
    private int greenTime = 4;
    private int phaseIndex = 0;

    public FixedTimeController(Lane[] src) { super("Junction", src); }
    public String modeName() { return "Fixed-Time"; }

    @Override
    public void runControl() {
        while (vehiclesLeft()) {
            serve(lanes[phaseIndex], greenTime);
            phaseIndex = (phaseIndex + 1) % 4;      // circular rotation N -> E -> S -> W -> N
        }
    }
}

// ---------------------------------------------------------------- Greedy adaptive
class GreedyController extends SignalController {
    public GreedyController(Lane[] src) { super("Junction", src); }
    public String modeName() { return "Greedy"; }

    @Override
    public void runControl() {
        while (vehiclesLeft()) {
            Lane best = greedyLane();               // busiest lane first
            if (best == null) { waitForArrival(); continue; }
            serve(best, Math.min(best.ready(time), maxGreen));
        }
    }
}

// ---------------------------------------------------------------- Emergency priority
class EmergencyController extends SignalController {
    public EmergencyController(Lane[] src) { super("Junction", src); }
    public String modeName() { return "Emergency"; }

    @Override
    public void runControl() {
        while (vehiclesLeft()) {
            PriorityQueue<Lane> heap = new PriorityQueue<>((a, b) -> {
                int pa = a.bestPriority(time), pb = b.bestPriority(time);
                if (pa != pb) return pa - pb;                       // most urgent vehicle first
                return b.ready(time) - a.ready(time);               // tie: busiest lane
            });
            for (Lane l : lanes) if (l.ready(time) > 0) heap.add(l);
            Lane best = heap.poll();
            if (best == null) { waitForArrival(); continue; }
            int p = best.bestPriority(time);
            int units = (p < 4) ? best.positionOf(p, time)           // green until it has passed
                                : Math.min(best.ready(time), maxGreen);
            serve(best, units);
        }
    }

    public void showRoute(RoadGraph g, String a, String b) {
        List<String> path = PathFinder.shortestPath(g, a, b);
        if (path == null) { System.out.println("No route found."); return; }
        System.out.println("Fastest Route : " + String.join(" -> ", path));
        System.out.println("Travel Time   : " + PathFinder.lastCost + " units");
        if (path.size() > 2)
            System.out.println("Green corridor at: " + String.join(", ", path.subList(1, path.size() - 1)));
    }
}

// ---------------------------------------------------------------- Main
public class TrafficSignalSimulator {
    private static RoadGraph graph = new RoadGraph();
    private static Lane[] lanes = newLanes();
    private static Scanner sc = new Scanner(System.in);
    private static int counter = 0;

    private static Lane[] newLanes() {
        return new Lane[]{new Lane('N'), new Lane('E'), new Lane('S'), new Lane('W')};
    }

    private static int laneIndex(char c) { return "NESW".indexOf(Character.toUpperCase(c)); }

    private static int readInt(String msg) {
        while (true) {
            System.out.print(msg);
            try { return Integer.parseInt(sc.nextLine().trim()); }
            catch (NumberFormatException ex) { System.out.println("Please enter a number."); }
        }
    }

    private static void addVehicles() {
        System.out.print("Enter Lane (N/E/S/W): ");
        int li = laneIndex(sc.nextLine().trim().toUpperCase().charAt(0));
        if (li < 0) { System.out.println("Invalid lane."); return; }
        int n = readInt("Number of Vehicles: ");
        int at = readInt("Arrival Time: ");
        int pr = readInt("Highest Priority (1-4): ");
        for (int i = 0; i < n; i++) {
            // the most urgent vehicle goes at the rear of the lane, the rest are cars
            int p = (i == n - 1 && pr < 4) ? pr : 4;
            counter++;
            if (!lanes[li].addVehicle(new Vehicle("V" + counter, "NESW".charAt(li), at, p))) {
                System.out.println("Lane is full!");
                break;
            }
        }
        System.out.println("Vehicles added successfully!");
    }

    private static void viewLanes() {
        System.out.println("LANE  VEH  AT  Pr");
        System.out.println("--------------------");
        int total = 0;
        for (Lane l : lanes) {
            int at = Integer.MAX_VALUE, pr = 99;
            for (Vehicle v : l.snapshot()) { at = Math.min(at, v.getArrivalTime()); pr = Math.min(pr, v.getPriority()); }
            if (l.size() == 0) System.out.println(String.format("%-6s%-5d-    -", l.getName(), 0));
            else System.out.println(String.format("%-6s%-5d%-4d%d", l.getName(), l.size(), at, pr));
            total += l.size();
        }
        System.out.println("Total vehicles: " + total);
    }

    private static SignalController make(int mode) {
        if (mode == 1) return new FixedTimeController(lanes);
        if (mode == 2) return new GreedyController(lanes);
        return new EmergencyController(lanes);
    }

    private static void runMode() {
        int mode = readInt("Select Mode (1-Fixed-Time 2-Greedy 3-Emergency): ");
        if (mode < 1 || mode > 3) { System.out.println("Invalid mode."); return; }
        SignalController c = make(mode);
        c.runControl();
        System.out.println();
        c.printTimeline();
        System.out.println();
        c.printResults();
    }

    private static void compareAll() {
        System.out.println("====== MODE COMPARISON ======");
        System.out.println(String.format("%-14s%-9s%-7s%-6s%s", "Mode", "Avg WT", "Clear", "Idle", "Amb WT"));
        System.out.println("------------------------------------------");
        String best = ""; double bw = 1e9;
        for (int m = 1; m <= 3; m++) {
            SignalController c = make(m);
            c.runControl();
            int amb = c.getAmbulanceWait();
            System.out.println(String.format("%-14s%-9.2f%-7d%-6d%s", c.modeName(), c.getAvgWaitingTime(),
                    c.getClearanceTime(), c.getTotalIdleGreen(), amb < 0 ? "-" : String.valueOf(amb)));
        }
    }

    private static void buildMap() {
        int n = readInt("Number of roads: ");
        for (int i = 0; i < n; i++) {
            System.out.print("Road " + (i + 1) + " (from to time): ");
            String[] p = sc.nextLine().trim().split("\\s+");
            if (p.length < 3) { System.out.println("Invalid input."); i--; continue; }
            graph.addRoad(p[0].toUpperCase(), p[1].toUpperCase(), Integer.parseInt(p[2]));
        }
        graph.display();
    }

    private static void findRoute() {
        graph.display();
        System.out.print("Source: ");
        String a = sc.nextLine().trim().toUpperCase();
        System.out.print("Destination: ");
        String b = sc.nextLine().trim().toUpperCase();
        if (!graph.has(a) || !graph.has(b)) { System.out.println("Unknown intersection."); return; }
        new EmergencyController(lanes).showRoute(graph, a, b);
    }

    private static void loadSample() {
        lanes = newLanes();
        counter = 0;
        int[][] data = {{5, 0}, {4, 0}, {8, 0}, {6, 0}};          // vehicles per lane
        int[] urgentPos = {-1, 3, -1, 4};                          // 1-based position of urgent vehicle
        int[] urgentPr = {4, 1, 4, 3};
        for (int i = 0; i < 4; i++)
            for (int k = 1; k <= data[i][0]; k++) {
                counter++;
                int p = (k == urgentPos[i]) ? urgentPr[i] : 4;
                lanes[i].addVehicle(new Vehicle("V" + counter, "NESW".charAt(i), 0, p));
            }
        graph = new RoadGraph();
        String[][] roads = {{"A","B","4"},{"A","C","2"},{"B","C","1"},{"B","D","5"},{"C","D","8"},
                            {"C","E","10"},{"D","E","2"},{"D","F","6"},{"E","F","3"}};
        for (String[] r : roads) graph.addRoad(r[0], r[1], Integer.parseInt(r[2]));
        System.out.println("Sample traffic and road map loaded.");
    }

    public static void main(String[] args) {
        int choice;
        do {
            System.out.println();
            System.out.println("====== TRAFFIC SIGNAL SIMULATOR ======");
            System.out.println("1. Add Vehicles");
            System.out.println("2. View Lane Status");
            System.out.println("3. Run Signal Control");
            System.out.println("4. Compare All Modes");
            System.out.println("5. Build Road Map");
            System.out.println("6. Find Emergency Route");
            System.out.println("7. Load Sample Data");
            System.out.println("0. Exit");
            choice = readInt("Enter choice: ");
            switch (choice) {
                case 1: addVehicles(); break;
                case 2: viewLanes(); break;
                case 3: runMode(); break;
                case 4: compareAll(); break;
                case 5: buildMap(); break;
                case 6: findRoute(); break;
                case 7: loadSample(); break;
                case 0: System.out.println("Thank you!"); break;
                default: System.out.println("Invalid choice.");
            }
        } while (choice != 0);
    }
}