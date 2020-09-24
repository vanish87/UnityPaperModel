using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;
using UnityTools.Common;
using UnityTools.Debuging;
using UnityTools.Debuging.EditorTool;

namespace UnityPaperModel
{
    [System.Serializable]
    public class DualGraph
    {
        public static Mesh WeldVertices(Mesh aMesh, float aMaxDelta = 0.01f) {
             var verts = aMesh.vertices;
             var normals = aMesh.normals;
             var uvs = aMesh.uv;
             Dictionary<Vector3, int> duplicateHashTable = new Dictionary<Vector3, int>();
             List<int> newVerts = new List<int>();
             int[] map = new int[verts.Length];
 
             //create mapping and find duplicates, dictionaries are like hashtables, mean fast
             for (int i = 0; i < verts.Length; i++) {
                 if (!duplicateHashTable.ContainsKey(verts[i])) {
                     duplicateHashTable.Add(verts[i], newVerts.Count);
                     map[i] = newVerts.Count;
                     newVerts.Add(i);
                 }
                 else {
                     map[i] = duplicateHashTable[verts[i]];
                 }
             }
 
             // create new vertices
             var verts2 = new Vector3[newVerts.Count];
             var normals2 = new Vector3[newVerts.Count];
             var uvs2 = new Vector2[newVerts.Count];
             for (int i = 0; i < newVerts.Count; i++) {
                 int a = newVerts[i];
                 verts2[i] = verts[a];
                 normals2[i] = normals[a];
                 uvs2[i] = uvs[a];
             }
             // map the triangle to the new vertices
             var tris = aMesh.triangles;
             for (int i = 0; i < tris.Length; i++) {
                 tris[i] = map[tris[i]];
             }
             aMesh.triangles = tris;
             aMesh.vertices = verts2;
             aMesh.normals = normals2;
             aMesh.uv = uvs2;
 
             aMesh.RecalculateBounds();
             aMesh.RecalculateNormals();
 
             return aMesh;
         }
        public void UpdateGraph(int ia, int ib, Vector3[] vertices, GraphAdj<Vertex, Edge> graph, GraphAdj<Face, DualEdge> dualGraph, Face face)
        {
            var va = vertices[ia];
            var vb = vertices[ib];
            var gNodes = graph.Nodes.ToList();
            var dNodes = dualGraph.Nodes.ToList();
            var e0 = graph.GetEdge(ia, ib);
            if (e0 == null)
            {
                e0 = new Edge();
                e0.Start = gNodes[ia];
                e0.End = gNodes[ib];
                e0.Start.position = va;
                e0.End.position = vb;

                var de0 = new DualEdge();
                de0.Start = face;
                e0.dualEdge = de0;

                graph.AddEdge(ia, ib, e0);
            }
            else
            {
                LogTool.AssertNotNull(e0.dualEdge);
                var de = e0.dualEdge;
                if (de.Start == null) de.Start = face;
                else de.End = face;

                LogTool.AssertNotNull(de.Start);
                LogTool.AssertNotNull(de.End);

                dualGraph.AddEdge(de.Start, de.End, de);
            }
        }
        public GraphAdj<Face, DualEdge> GenerateGraph(Mesh mesh)
        {
            LogTool.AssertIsTrue(mesh != null);
            LogTool.LogAssertIsTrue(mesh.GetTopology(0) == MeshTopology.Triangles, "Only support triangle now");
            LogTool.LogAssertIsTrue(mesh.subMeshCount == 1, "Only support 1 sub mesh now");

            WeldVertices(mesh, 0.01f);

            var vCount = mesh.vertexCount;
            var fCount = mesh.triangles.Length / 3;
            this.graph = new GraphAdj<Vertex, Edge>(vCount);
            this.dualGraph = new GraphAdj<Face, DualEdge>(fCount);

            var gNodes = this.graph.Nodes.ToList();
            var dNodes = this.dualGraph.Nodes.ToList();

            for (var i = 0; i < fCount; ++i)
            {
                var ia = i * 3;
                var ib = i * 3 + 1;
                var ic = i * 3 + 2;
                var a = mesh.triangles[ia];
                var b = mesh.triangles[ib];
                var c = mesh.triangles[ic];
                var va = mesh.vertices[a];
                var vb = mesh.vertices[b];
                var vc = mesh.vertices[c];

                var vl = new List<float3>();
                vl.Add(va);
                vl.Add(vb);
                vl.Add(vc);

                var face = dNodes[i];
                face.Init(vl);

                UpdateGraph(a, b, mesh.vertices, graph, dualGraph, face);
                UpdateGraph(b, c, mesh.vertices, graph, dualGraph, face);
                UpdateGraph(c, a, mesh.vertices, graph, dualGraph, face);

            }

            return dualGraph;
        }

        [System.Serializable]
        public class Face : INode
        {
            public int Index { get; set; }
            public float3 Center;

            public void Init(List<float3> vertices)
            {
                LogTool.AssertIsTrue(vertices != null);

                foreach (var v in vertices)
                {
                    this.Center += v;
                }

                this.Center /= vertices.Count;
            }

        }
        [System.Serializable]
        public class Vertex : INode
        {
            public int Index { get; set; }

            public float3 position;
        }
        [System.Serializable]
        public class Edge : Segment<Vertex>, IEdge
        {
            public DualEdge dualEdge;
        }
        [System.Serializable]
        public class DualEdge : Segment<Face>, IEdge
        {
            //large weight value is more likely to be cut
            public float Weight { get => this.Weight; }
            protected float weight = 0;
        }

        public GraphAdj<Face, DualEdge> GetDualGraph => this.dualGraph;
        protected GraphAdj<Face, DualEdge> dualGraph;
        protected GraphAdj<Vertex, Edge> graph;

        public DualGraph(Mesh mesh)
        {
            this.dualGraph = GenerateGraph(mesh);
        }
        public static void Save(DualGraph graph)
        {
            var path = Path.Combine(Application.streamingAssetsPath, "Cat.graph");
            FileTool.Write(path,graph);
        }
        public static DualGraph Load()
        {
            var path = Path.Combine(Application.streamingAssetsPath, "Cat.graph");
            var ret = FileTool.Read<DualGraph>(path);
            return ret;
        }

        public void OnDrawGizmos()
        {
            if(this.dualGraph == null || this.graph == null) return;

            using (new GizmosScope(Color.cyan, Matrix4x4.identity))
            {
                foreach (var n in this.dualGraph.Nodes)
                {
                    Gizmos.DrawSphere(n.Center, 0.01f);
                }

                foreach (var e in this.dualGraph.GetEdges())
                {
                    Gizmos.DrawLine(e.Start.Center, e.End.Center);
                }
            }
            using (new GizmosScope(Color.gray, Matrix4x4.identity))
            {
                foreach (var n in this.graph.Nodes)
                {
                    Gizmos.DrawSphere(n.position, 0.01f);
                }

                foreach (var e in this.graph.GetEdges())
                {
                    Gizmos.DrawLine(e.Start.position, e.End.position);
                }
            }
        }
    }
}