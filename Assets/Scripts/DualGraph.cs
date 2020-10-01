using System.Collections.Generic;
using System.IO;
using Unity.Mathematics;
using UnityEngine;
using UnityTools.Common;
using UnityTools.Debuging;

namespace UnityPaperModel
{
    public class VertexGraph : NewGraph<VertexGraph.Vertex, VertexGraph.Edge, VertexGraph.GraphFactory>
    {
        public class GraphFactory : IGraphFactory
        {
            public IEdge CreateEdge(IVertex v1, IVertex v2, bool isDirectional = false)
            {
                return new Edge() { Vertex = v1, OtherVertex = v2 };
            }

            public IVertex CreateVertex()
            {
                return new Vertex();
            }
        }
        [System.Serializable]
        public class Vertex : IVertex
        {
            public float3 Position { get => this.position; set => this.position = value; }
            protected float3 position;
            public object Clone()
            {
                return new Vertex() { position = this.Position };
            }
            public override bool Equals(object other)
            {
                if(other is IVertex) return this.Equals(other as IVertex);
                return base.Equals(other);
            }

            public bool Equals(IVertex other)
            {
                var with = other as Vertex;
                if(with == null) return false;
                return math.distance(this.Position, with.Position) < 0.001f;
            }

            public override int GetHashCode()
            {
                return string.Format("{0:F2}{1:F2}{2:F2}", this.Position.x, this.Position.y, this.Position.z).GetHashCode();
            }
        }

        [System.Serializable]
        public class Edge : DefaultEdge
        {
            public override string ToString()
            {
                return (this.Vertex as Vertex).Position + " " + (this.OtherVertex as Vertex).Position;
            }
        }

        public void OnDrawGizmos()
        {
            foreach(var v in this.Vertices)
            {
                Gizmos.DrawSphere(v.Position, 0.01f);
            }

            foreach (var e in this.Edges)
            {
                var p1 = e.Vertex as Vertex;
                var p2 = e.OtherVertex as Vertex;
                Gizmos.DrawLine(p1.Position, p2.Position);
            }
        }

    }

    public class NewDualGraph:NewGraph<NewDualGraph.Face, NewDualGraph.Edge, NewDualGraph.GraphFactory>
    {
        public class GraphFactory : IGraphFactory
        {
            public IEdge CreateEdge(IVertex v1, IVertex v2, bool isDirectional = false)
            {
                return new Edge() { Vertex = v1, OtherVertex = v2 };
            }

            public IVertex CreateVertex()
            {
                return new Face();
            }
        }
        public class Face : IVertex
        {
            public float3 Center => this.center; 
            protected HashSet<VertexGraph.Vertex> vertices = new HashSet<VertexGraph.Vertex>();
            protected float3 sum = float3.zero;
            protected float3 center = float3.zero;
            
            public void AddVertex(VertexGraph.Vertex v)
            {
                if(this.vertices.Add(v))
                {
                    this.sum += v.Position;
                }
                this.center = this.sum / this.vertices.Count;
            }
            public bool Contains(VertexGraph.Vertex v)
            {
                return this.vertices.Contains(v);
                // Debug.Log(v.GetHashCode());
                // foreach(var vi in this.vertices)
                // {
                //     if(vi.Equals(v)) 
                //     {
                //         Debug.Log(vi.GetHashCode());

                //         Debug.Log(this.vertices.Contains(v));

                //         return true;
                //     }
                // }
                // return false;
            }
            public object Clone()
            {
                var ret = new Face() { sum = this.sum };
                foreach(var v in this.vertices)
                {
                    ret.vertices.Add(v.Clone() as VertexGraph.Vertex);
                }
                return ret;
            }
            public override bool Equals(object other)
            {
                if(other is IVertex) return this.Equals(other as IVertex);
                return base.Equals(other);
            }

            public bool Equals(IVertex other)
            {
                var f = other as Face;
                if(f == null) return false;

                if(math.distance(this.Center, f.Center) < 0.001f) return true;

                //Note not sure this is correct
                return this.vertices.GetHashCode() == f.vertices.GetHashCode();
                // foreach(var v in this.vertices)
                // {
                //     if(f.Contains(v) == false) return false;
                // }
                // return true;
            }
            public override int GetHashCode()
            {
                return string.Format("{0:F2}{1:F2}{2:F2}", this.Center.x, this.Center.y, this.Center.z).GetHashCode();
            }
        }

        public class Edge:DefaultEdge
        {

        }

        public Face FindFaceContainsVertexOtherThan(Face face, VertexGraph.Vertex v1, VertexGraph.Vertex v2)
        {
            foreach(var f in this.Vertices)
            {
                if(f != face && f.Contains(v1) && f.Contains(v2))
                {
                    return f;
                }
            }
            return default;
        }
        public void OnDrawGizmos()
        {
            foreach(var v in this.Vertices)
            {
                Gizmos.DrawSphere(v.Center, 0.01f);
                // var p0 = v.Edges[0].Vertex as VertexGraph.Vertex;
                // var p1 = v.Edges[0].OtherVertex as VertexGraph.Vertex;
                // Gizmos.DrawLine(p0.Position, p1.Position);
                // p0 = v.Edges[1].Vertex as VertexGraph.Vertex;
                // p1 = v.Edges[1].OtherVertex as VertexGraph.Vertex;
                // Gizmos.DrawLine(p0.Position, p1.Position);
                // p0 = v.Edges[2].Vertex as VertexGraph.Vertex;
                // p1 = v.Edges[2].OtherVertex as VertexGraph.Vertex;
                // Gizmos.DrawLine(p0.Position, p1.Position);
            }


            foreach (var e in this.Edges)
            {
                var p1 = e.Vertex as Face;
                var p2 = e.OtherVertex as Face;
                Gizmos.DrawLine(p1.Center, p2.Center);
            }
        }
    }
    [System.Serializable]
    public class DualGraph
    {
        public static Mesh WeldVertices(Mesh aMesh, float aMaxDelta = 0.01f)
        {
            var verts = aMesh.vertices;
            var normals = aMesh.normals;
            var uvs = aMesh.uv;
            Dictionary<Vector3, int> duplicateHashTable = new Dictionary<Vector3, int>();
            List<int> newVerts = new List<int>();
            int[] map = new int[verts.Length];

            //create mapping and find duplicates, dictionaries are like hashtables, mean fast
            for (int i = 0; i < verts.Length; i++)
            {
                if (!duplicateHashTable.ContainsKey(verts[i]))
                {
                    duplicateHashTable.Add(verts[i], newVerts.Count);
                    map[i] = newVerts.Count;
                    newVerts.Add(i);
                }
                else
                {
                    map[i] = duplicateHashTable[verts[i]];
                }
            }

            // create new vertices
            var verts2 = new Vector3[newVerts.Count];
            var normals2 = new Vector3[newVerts.Count];
            //  var uvs2 = new Vector2[newVerts.Count];
            for (int i = 0; i < newVerts.Count; i++)
            {
                int a = newVerts[i];
                verts2[i] = verts[a];
                normals2[i] = normals[a];
                //  uvs2[i] = uvs[a];
            }
            // map the triangle to the new vertices
            var tris = aMesh.triangles;
            for (int i = 0; i < tris.Length; i++)
            {
                tris[i] = map[tris[i]];
            }
            aMesh.triangles = tris;
            aMesh.vertices = verts2;
            aMesh.normals = normals2;
            //  aMesh.uv = uvs2;

            aMesh.RecalculateBounds();
            aMesh.RecalculateNormals();

            return aMesh;
        }
        // public void UpdateGraph(int ia, int ib, Vector3[] vertices, GraphAdj<Vertex, Edge> graph, GraphAdj<Face, DualEdge> dualGraph, Face face)
        // {
        //     var va = vertices[ia];
        //     var vb = vertices[ib];
        //     var gNodes = graph.Nodes.ToList();
        //     var dNodes = dualGraph.Nodes.ToList();
        //     var e0 = graph.GetEdge(ia, ib);
        //     if (e0 == null)
        //     {
        //         e0 = new Edge();
        //         e0.Start = gNodes[ia];
        //         e0.End = gNodes[ib];
        //         e0.Start.position = va;
        //         e0.End.position = vb;

        //         var de0 = new DualEdge();
        //         de0.Start = face;
        //         e0.dualEdge = de0;

        //         graph.AddEdge(ia, ib, e0);
        //     }
        //     else
        //     {
        //         LogTool.AssertNotNull(e0.dualEdge);
        //         var de = e0.dualEdge;
        //         if (de.Start == null) de.Start = face;
        //         else de.End = face;

        //         LogTool.AssertNotNull(de.Start);
        //         LogTool.AssertNotNull(de.End);

        //         de.edge = e0;

        //         dualGraph.AddEdge(de.Start, de.End, de);
        //     }
        // }
        // public GraphAdj<Face, DualEdge> GenerateGraph(Mesh mesh)
        // {
        //     LogTool.AssertIsTrue(mesh != null);
        //     LogTool.LogAssertIsTrue(mesh.GetTopology(0) == MeshTopology.Triangles, "Only support triangle now");
        //     LogTool.LogAssertIsTrue(mesh.subMeshCount == 1, "Only support 1 sub mesh now");

        //     WeldVertices(mesh, 0.01f);

        //     var vCount = mesh.vertexCount;
        //     var fCount = mesh.triangles.Length / 3;
        //     this.graph = new GraphAdj<Vertex, Edge>(vCount);
        //     this.dualGraph = new GraphAdj<Face, DualEdge>(fCount);

        //     var gNodes = this.graph.Nodes.ToList();
        //     var dNodes = this.dualGraph.Nodes.ToList();

        //     for (var i = 0; i < fCount; ++i)
        //     {
        //         var ia = i * 3;
        //         var ib = i * 3 + 1;
        //         var ic = i * 3 + 2;
        //         var a = mesh.triangles[ia];
        //         var b = mesh.triangles[ib];
        //         var c = mesh.triangles[ic];
        //         var va = mesh.vertices[a];
        //         var vb = mesh.vertices[b];
        //         var vc = mesh.vertices[c];

        //         var vl = new List<float3>();
        //         vl.Add(va);
        //         vl.Add(vb);
        //         vl.Add(vc);

        //         var face = dNodes[i];
        //         face.Init(vl);

        //         UpdateGraph(a, b, mesh.vertices, graph, dualGraph, face);
        //         UpdateGraph(b, c, mesh.vertices, graph, dualGraph, face);
        //         UpdateGraph(c, a, mesh.vertices, graph, dualGraph, face);

        //     }

        //     return dualGraph;
        // }

        [System.Serializable]
        public class Face : IndexVertex
        {
            public float3 Center;
            public List<float3> world;
            public List<float3> newWorld;
            [System.NonSerialized]public Matrix4x4 local;

            public float3 Normal
            {
                get
                {
                    this.CalCenter();
                    return math.normalize(math.cross(world[1] - world[0], world[2] - world[0]));

                }
            }

            public void Init(List<float3> vertices)
            {
                LogTool.AssertIsTrue(vertices != null);

                this.world = vertices;
                this.newWorld = new List<float3>();
                this.newWorld.AddRange(vertices);
                this.local = Matrix4x4.identity;
            }

            protected void CalCenter()
            {
                this.Center = 0;
                foreach (var v in this.world)
                {
                    this.Center += v;
                }

                this.Center /= this.world.Count;

            }
            
        }
        [System.Serializable]
        public class Vertex : IndexVertex
        {
            public float3 position;
        }
        [System.Serializable]
        public class Edge : DefaultEdge
        {
            public DualEdge dualEdge;

        }
        [System.Serializable]
        public class DualEdge : DefaultEdge
        {
            //large weight value is more likely to be cut
            public float Weight { get => this.Weight; }
            protected float weight = 0;


            public Edge edge;

        }

        protected NewGraph<Face, DualEdge, IndexGraphFactory> dualGraph;

        protected NewGraph<Vertex, Edge, IndexGraphFactory> graph;
        public DualGraph(Mesh mesh)
        {
            // this.dualGraph = GenerateGraph(mesh);
        }
        public static void Save(DualGraph graph)
        {
            var path = Path.Combine(Application.streamingAssetsPath, "Cat.graph");
            FileTool.Write(path, graph);
        }
        public static DualGraph Load()
        {
            var path = Path.Combine(Application.streamingAssetsPath, "Cat.graph");
            var ret = FileTool.Read<DualGraph>(path);
            return ret;
        }

        public void OnDrawGizmos()
        {
            if (this.dualGraph == null || this.graph == null) return;

            // using (new GizmosScope(Color.cyan, Matrix4x4.identity))
            // {
            //     foreach (var n in this.dualGraph.Nodes)
            //     {
            //         Gizmos.DrawSphere(n.Center, 0.01f);
            //     }

            //     foreach (var e in this.dualGraph.GetEdges())
            //     {
            //         Gizmos.DrawLine(e.Start.Center, e.End.Center);
            //     }
            // }
            // using (new GizmosScope(Color.gray, Matrix4x4.identity))
            // {
            //     foreach (var n in this.graph.Nodes)
            //     {
            //         Gizmos.DrawSphere(n.position, 0.01f);
            //     }

            //     foreach (var e in this.graph.GetEdges())
            //     {
            //         Gizmos.DrawLine(e.Start.position, e.End.position);
            //     }
            // }
        }
    }
}