// Author: OpenAI and Bob King (aka subbob)
// Date: 19 Dec 2023
//
// Conversation Source: https://chat.openai.com/share/6e67e70f-7999-4acc-9c6d-14f461950add

using System;
using System.Collections.Generic;
using System.Linq;

public class Location
{
    public string Name { get; set; }
    public int X { get; set; }
    public int Y { get; set; }
    public int Z { get; set; }

    // Constructor to initialize the properties
    public Location(string name, int x, int y, int z)
    {
        Name = name;
        X = x;
        Y = y;
        Z = z;
    }

    // Method to display location information
    public void DisplayLocation()
    {
        Console.WriteLine($"{Name} Location: ({X}, {Y}, {Z})");
    }

    // Method to calculate Euclidean distance between two locations
    public double CalculateDistance(Location other)
    {
        int dx = X - other.X;
        int dy = Y - other.Y;
        int dz = Z - other.Z;
        return Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }
}

public class Sphere
{
    public Location Origin { get; set; }
    public int Radius { get; set; }

    // Constructor to initialize the properties
    public Sphere(Location origin, int radius)
    {
        Origin = origin;
        Radius = radius;
    }

    // Method to display sphere information
    public void DisplaySphere()
    {
        Console.WriteLine($"Sphere around {Origin.Name} with radius {Radius} meters");
    }

    // Method to check if a location is inside the sphere
    public bool IsInsideSphere(Location location)
    {
        double distance = Origin.CalculateDistance(location);
        return distance <= Radius;
    }
}

class Program
{
	
	// Declare Locations as class  fields
	
	static Location ceresBody; 
	static Location vestaBody; 
	static Location t1Sector; 
	static Location pallasSector;
	static Location p1Sector;
	static Location greekCluster;
	static Location trojanCluster;
	static Location marsBody;
	static Location m1Sector;
	static Location m2Sector;
	static Location earthBody; 
	static Location e1Sector;
	static Location	e2Sector;
	static Location c1Sector;
	static Location c2Sector;
	static Location jupiterBody; 
	static Location saturnBody;
	static Location uranusBody;
	static Location lunaBody;
	static Location ganymedeBody;
	static Location phobosBody;
    
	static void Main()
    {
        // ... (previous code remains unchanged)
        // Declare instances of the Location class for each location
        ceresBody = new Location("CeresBody", 0, 2350000, 100000);
	    vestaBody = new Location("VestaBody", -1400000, 1000000, -50000);
	    t1Sector = new Location("T1Sector", 1725000, 0, 0);
	    pallasSector = new Location("PallasSector", 0, -1725000, 100000);
	    p1Sector = new Location("P1Sector", -800000, -1550000, 0);
	    greekCluster = new Location("GreekCluster", -1600000, -2700000, -334490);
	    trojanCluster = new Location("TrojanCluster", 1600000, -2700000, 445350);
	    marsBody = new Location("MarsBody", 750000, -750000, -80000);
	    m1Sector = new Location("M1Sector", 800000, -1550000, 0);
	    m2Sector = new Location("M2Sector", 1550000, -800000, 0);
	    earthBody = new Location("EarthBody", -800000, 100000, 80000);
		e1Sector  = new Location("E1Sector", -1725000, 0, 0);
		e2Sector = new Location("E2Sector", -1500000, -900000, 0);
	    c1Sector = new Location("C1Sector", 600000, 1600000, 0);
	    c2Sector = new Location("C2Sector", -600000, 1600000, 0);
	    jupiterBody = new Location("JupiterBody", 0, -3200000, 0);
	    saturnBody = new Location("SaturnBody", 2800000, 2000000, -100000);
	    uranusBody = new Location("UranusBody", -4200000, 3000000, -250000);
	    lunaBody = new Location("LunaBody", -650000, 225000, 0);
	    ganymedeBody = new Location("GanymedeBody", -400000, -3100000, -300000);
	    phobosBody = new Location("PhobosBody", 650000, -650000, -250000);

        // Display information for each location
        ceresBody.DisplayLocation();
        vestaBody.DisplayLocation();
        t1Sector.DisplayLocation();
        pallasSector.DisplayLocation();
        p1Sector.DisplayLocation();
        greekCluster.DisplayLocation();
        trojanCluster.DisplayLocation();
        marsBody.DisplayLocation();
        m1Sector.DisplayLocation();
        m2Sector.DisplayLocation();
        earthBody.DisplayLocation();
        e1Sector.DisplayLocation();
        e2Sector.DisplayLocation();
        c1Sector.DisplayLocation();
        c2Sector.DisplayLocation();
        jupiterBody.DisplayLocation();
        saturnBody.DisplayLocation();
        uranusBody.DisplayLocation();
        lunaBody.DisplayLocation();
        ganymedeBody.DisplayLocation();
        phobosBody.DisplayLocation();

        // Example usage of route planning
        Location origin = new Location("CeresStation", -9793, 2325984, 84772);// Replace with the desired origin
        Location destination = new Location("VestaStation", -1413375, 986500, -50041); // Replace with the desired destination

        List<Location> route = FindOptimalRoute(origin, destination);

        // Display the route
        Console.WriteLine("Optimal Route:");
        foreach (var point in route)
        {
            point.DisplayLocation();
        }
    }

    static List<Location> FindOptimalRoute(Location origin, Location destination)
    {
        // A* algorithm implementation
        HashSet<Location> closedSet = new HashSet<Location>();
        HashSet<Location> openSet = new HashSet<Location> { origin };
        Dictionary<Location, Location> cameFrom = new Dictionary<Location, Location>();
        Dictionary<Location, double> gScore = new Dictionary<Location, double> { { origin, 0 } };
        Dictionary<Location, double> fScore = new Dictionary<Location, double> { { origin, origin.CalculateDistance(destination) } };

        while (openSet.Any())
        {
            Location current = openSet.OrderBy(loc => fScore[loc]).First();

            if (current == destination)
            {
                return ReconstructPath(cameFrom, current);
            }

            openSet.Remove(current);
            closedSet.Add(current);

            foreach (var neighbor in GetNeighbors(current))
            {
                if (closedSet.Contains(neighbor))
                    continue;

                double tentativeGScore = gScore[current] + current.CalculateDistance(neighbor);

                if (!openSet.Contains(neighbor) || tentativeGScore < gScore[neighbor])
                {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeGScore;
                    fScore[neighbor] = tentativeGScore + neighbor.CalculateDistance(destination);

                    if (!openSet.Contains(neighbor))
                        openSet.Add(neighbor);
                }
            }
        }

        // No path found
        return null;
    }

    static List<Location> ReconstructPath(Dictionary<Location, Location> cameFrom, Location current)
    {
        List<Location> path = new List<Location> { current };

        while (cameFrom.ContainsKey(current))
        {
            current = cameFrom[current];
            path.Insert(0, current);
        }

        return path;
    }

    static IEnumerable<Location> GetNeighbors(Location location)
    {
        // Placeholder method to get neighboring locations
        // Replace this with logic based on the provided data (location and sphere instances)

        // For simplicity, assume all adjacent locations are valid neighbors
        // You need to adapt this to your specific spatial layout

        List<Location> neighbors = new List<Location>
        {
            new Location(location.Name, location.X + 1, location.Y, location.Z),
            new Location(location.Name, location.X - 1, location.Y, location.Z),
            new Location(location.Name, location.X, location.Y + 1, location.Z),
            new Location(location.Name, location.X, location.Y - 1, location.Z),
            new Location(location.Name, location.X, location.Y, location.Z + 1),
            new Location(location.Name, location.X, location.Y, location.Z - 1)
        };

        // Filter out neighbors that are inside spheres
        foreach (var sphere in spheres)
        {
            neighbors = neighbors.Where(neighbor => !sphere.IsInsideSphere(neighbor)).ToList();
        }

        return neighbors;
    }

    static List<Sphere> spheres = new List<Sphere>
    {
        new Sphere(earthBody, 400000),
        new Sphere(t1Sector, 300000),
        new Sphere(pallasSector, 300000),
        new Sphere(e1Sector, 300000),
        new Sphere(e2Sector, 300000),
        new Sphere(greekCluster, 400000),
        new Sphere(trojanCluster, 400000),
        new Sphere(marsBody, 400000),
        new Sphere(m1Sector, 300000),
        new Sphere(m2Sector, 300000),
        new Sphere(ceresBody, 400000),
        new Sphere(c1Sector, 300000),
        new Sphere(c2Sector, 300000),
        new Sphere(jupiterBody, 750000),
        new Sphere(saturnBody, 750000),
        // Add more spheres as needed
    };
}
