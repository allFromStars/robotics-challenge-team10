bool calculateWeightedPath() {
  int costGrid[GRID_HEIGHT][GRID_WIDTH];
  bool visited[GRID_HEIGHT][GRID_WIDTH];

  for (int y = 0; y < GRID_HEIGHT; y++) {
    for (int x = 0; x < GRID_WIDTH; x++) {
      costGrid[y][x] = 9999;
      visited[y][x] = false; // Initialise all as unvisited
    }
  }

  Coordinate queue[400];
  int head = 0, tail = 0;

  Coordinate target = robotInfo.finalDestination;
  if (target.x < 0 || target.x >= GRID_WIDTH || target.y < 0 || target.y >= GRID_HEIGHT) {
    Serial.println("[ERROR] Pathfinding target is outside the map!");
    return false;
  }

  costGrid[target.y][target.x] = 0; 
  queue[tail++] = target;

  const int LINE_PREFERENCE_MULTIPLIER = 35; 

  while (head < tail) {

    int minIdx = head;
    int minCost = costGrid[queue[head].y][queue[head].x];
    
    for (int i = head + 1; i < tail; i++) {
      int c = costGrid[queue[i].y][queue[i].x];
      if (c < minCost) {
        minCost = c;
        minIdx = i;
      }
    }

    // Swap the cheapest tile to the front of the line
    Coordinate temp = queue[head];
    queue[head] = queue[minIdx];
    queue[minIdx] = temp;
    // ----------------------------------------

    Coordinate current = queue[head++];
    int currentCost = costGrid[current.y][current.x];

 
    if (visited[current.y][current.x]) continue; 
    visited[current.y][current.x] = true;
    // ---------------------------------------

    for (int i = 0; i < 4; i++) {
      int nx = current.x + dx[i]; 
      int ny = current.y + dy[i]; 

      if (nx >= 0 && nx < GRID_WIDTH && ny >= 0 && ny < GRID_HEIGHT) {
       
        if (visited[ny][nx]) continue; 

        
        byte cellType = arenaMap[8 - ny][nx]; 

        if (cellType != 3) { 
          int stepWeight = (cellType == 1) ? 1 : (2 * LINE_PREFERENCE_MULTIPLIER);
          int newCost = currentCost + stepWeight;

          if (newCost < costGrid[ny][nx]) {
            costGrid[ny][nx] = newCost;
            
            // Queue boundary safety check
            if (tail < 400) {
              queue[tail++] = {nx, ny};
            } else {
              Serial.println("[ERROR] Pathfinding queue overflow!");
              return false;
            }
          }
        }
      }
    }
  }


  robotInfo.totalWaypoints = 0;
  robotInfo.currentWaypointIdx = 0;
  Coordinate tracer = robotInfo.currentPos;

  if (tracer.x < 0 || tracer.x >= GRID_WIDTH || tracer.y < 0 || tracer.y >= GRID_HEIGHT) {
    Serial.println("[ERROR] Robot position is outside the map!");
    return false;
  }
  
  if (costGrid[tracer.y][tracer.x] == 9999) return false; 

  while (!(tracer.x == target.x && tracer.y == target.y) && robotInfo.totalWaypoints < 30) {
    int bestDir = -1;
    int lowestCost = 99999;

    for (int i = 0; i < 4; i++) {
      int nx = tracer.x + dx[i];
      int ny = tracer.y + dy[i];

      if (nx >= 0 && nx < GRID_WIDTH && ny >= 0 && ny < GRID_HEIGHT) {
        if (costGrid[ny][nx] < lowestCost) {
          lowestCost = costGrid[ny][nx];
          bestDir = i;
        }
      }
    }

    if (bestDir == -1) break;

    tracer.x += dx[bestDir];
    tracer.y += dy[bestDir];
    

    robotInfo.waypoints[robotInfo.totalWaypoints++] = tracer;
  }

  if (!(tracer.x == target.x && tracer.y == target.y)) {
    Serial.println("[ERROR] Path exceeded waypoint storage!");
    return false;
  }

  return true;
}


String getHeadingString(Coordinate current, Coordinate next) {
  int diffX = next.x - current.x; 
  int diffY = next.y - current.y; 
  if (diffY > 0) return "NORTH ↑ (Y increases)";
  if (diffX > 0) return "EAST  → (X increases)";
  if (diffY < 0) return "SOUTH ↓ (Y decreases)";
  if (diffX < 0) return "WEST  ← (X decreases)";
  return "STAY";
}

float getTargetHeading(Coordinate current, Coordinate next) {
  int dx = next.x - current.x;
  int dy = next.y - current.y;

  // Yaw convention used by the IMU integration in this robot:
  // North = 0, East/right turn = -90, South = 180, West/left turn = +90.
  if (dy > 0)  return 0.0;    // Next node is North
  if (dx > 0)  return -90.0;  // Next node is East
  if (dy < 0)  return 180.0;  // Next node is South
  if (dx < 0)  return 90.0;   // Next node is West

  return sensors.yaw; //stay at current heading if coordinates match
}

void printVisualMap() {
  Serial.println("\n--- ARENA GRAPHICAL FEED ((0,0) AT BOTTOM LEFT) ---");
  
  // Loops backwards from 8 down to 0 so higher Y indices render at the top of the monitor screen
  for (int y = GRID_HEIGHT - 1; y >= 0; y--) {
    Serial.print("Y:"); Serial.print(y); Serial.print(" │ "); 
    
    for (int x = 0; x < GRID_WIDTH; x++) {
      // Pull data values passing through array adapter inversion
      byte mapCell = arenaMap[8 - y][x];
      char symbol = (mapCell == 1) ? '.' : ((mapCell == 3) ? 'X' : '-');
      
      for (int w = 0; w < robotInfo.totalWaypoints; w++) {
        if (robotInfo.waypoints[w].x == x && robotInfo.waypoints[w].y == y) symbol = '*';
      }
      
      if (x == robotInfo.currentPos.x && y == robotInfo.currentPos.y) symbol = 'R'; 
      if (x == robotInfo.finalDestination.x && y == robotInfo.finalDestination.y) symbol = 'T'; 
      
      Serial.print(symbol); Serial.print("  "); 
    }
    Serial.println();
  }
  
  Serial.print("    └──");
  for (int x = 0; x < GRID_WIDTH; x++) Serial.print("───");
  Serial.println();
  
  Serial.print("       "); 
  for (int x = 0; x < GRID_WIDTH; x++) {
    Serial.print(x); 
    Serial.print("  "); 
  }
  Serial.println("\n");
}

void moveForward() {

  int heading = (int)sensors.yaw;

  // 3. Update the coordinate grid coordinates by exactly 1 unit step
  switch (heading) {
    case 0:
      robotInfo.currentPos.y += 1; // Move North: Up the graph
      Serial.println("[DRIVE] Stepped 1 tile NORTH ↑");
      break;
      
    case 90:
      robotInfo.currentPos.x += 1; // Move East: Right across the graph
      Serial.println("[DRIVE] Stepped 1 tile EAST →");
      break;
      
    case 180:
      robotInfo.currentPos.y -= 1; // Move South: Down the graph
      Serial.println("[DRIVE] Stepped 1 tile SOUTH ↓");
      break;
      
    case 270:
      robotInfo.currentPos.x -= 1; // Move West: Left across the graph
      Serial.println("[DRIVE] Stepped 1 tile WEST ←");
      break;
      
    default:
      Serial.print("[DRIVE ERROR] Invalid target heading angle: ");
      break;
  }


  Serial.print("[POSITION] New Coordinate Registered: (X:");
  Serial.print(robotInfo.currentPos.x);
  Serial.print(", Y:");
  Serial.print(robotInfo.currentPos.y);
  Serial.println(")");
}
