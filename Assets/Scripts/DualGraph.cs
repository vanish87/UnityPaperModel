using System.Collections.Generic;
using System.IO;
using System.Linq;
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

            public INewGraph CreateGraph()
            {
                return new VertexGraph();
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
            public virtual object Clone()
            {
                return new Vertex() { position = this.Position };
            }
            public override bool Equals(object other)
            {
                if(other is IVertex) return this.Equals(other as IVertex);
                return base.Equals(other);
            }

            public virtual bool Equals(IVertex other)
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
            public override object Clone()
            {
                return new Edge(){ isDirectional = this.IsDirectional, Vertex = this.Vertex.Clone() as IVertex, OtherVertex = this.OtherVertex.Clone() as IVertex };
            }
            public override string ToString()
            {
                return (this.Vertex as Vertex).Position + " " + (this.OtherVertex as Vertex).Position;
            }
        }

        public static float3 GetNormal(IEdge e1, IEdge e2)
        {
            var v1 = (e1.OtherVertex as VertexGraph.Vertex).Position - (e1.Vertex as VertexGraph.Vertex).Position;
            var v2 = (e2.OtherVertex as VertexGraph.Vertex).Position - (e2.Vertex as VertexGraph.Vertex).Position;

            return math.normalize(math.cross(v1,v2));
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

    public class DualGraph:NewGraph<DualGraph.Face, DualGraph.Edge, DualGraph.GraphFactory>
    {
        public class GraphFactory : IGraphFactory
        {
            protected int currentID = 0;
            public IEdge CreateEdge(IVertex v1, IVertex v2, bool isDirectional = false)
            {
                return new Edge() { Vertex = v1, OtherVertex = v2 };
            }

            public INewGraph CreateGraph()
            {
                return new DualGraph();
            }

            public IVertex CreateVertex()
            {
                return new Face() { index = this.currentID++ };
            }
        }
        public class Face : IndexVertex
        {
            public float3 Center => this.center; 
            public float3 Normal 
            {
                get
                {
                    var e1 = this.edges.First();
                    var e2 = this.edges.Last();

                    return VertexGraph.GetNormal(e2, e1);
                }
            }

            public Matrix4x4 LocalMat { get => this.localRotationMat; set => this.localRotationMat = value; }
            protected HashSet<IEdge> edges = new HashSet<IEdge>();
            protected HashSet<IEdge> newEdges = new HashSet<IEdge>();
            protected float3 sum = float3.zero;
            protected float3 center = float3.zero;
            protected Matrix4x4 localRotationMat = Matrix4x4.identity;
            
            public void AddEdge(VertexGraph.Edge e)
            {
                if(this.edges.Add(e))
                {
                    this.sum += (e.Vertex as VertexGraph.Vertex).Position;
                    this.sum += (e.OtherVertex as VertexGraph.Vertex).Position;
                }
                this.center = this.sum / (this.edges.Count * 2);
            }

            public IEdge GetSharedEdgeWith(Face face)
            {
                foreach(var e in this.edges)
                {
                    if(face.Contains(e)) return e;
                }
                return default;                
            }
            public void Transform(Matrix4x4 mat)
            {
                this.TransformOrg(mat);
                return;
                this.newEdges.Clear();
                foreach(var v in this.edges)
                {
                    var v1 = mat.MultiplyPoint((v.Vertex as VertexGraph.Vertex).Position);
                    var v2 = mat.MultiplyPoint((v.OtherVertex as VertexGraph.Vertex).Position);
                    var newEdge = new VertexGraph.Edge() 
                    { 
                        Vertex = new VertexGraph.Vertex() { Position = v1 }, 
                        OtherVertex = new VertexGraph.Vertex() { Position = v2 } 
                    };
                    this.newEdges.Add(newEdge);
                }
            }

            protected void TransformOrg(Matrix4x4 mat)
            {
                var old = new List<IEdge>();
                foreach (var e in this.edges) old.Add(e.Clone() as IEdge);
                this.edges.Clear(); 
                this.sum = 0;
                this.center = 0;
                
                foreach(var v in old)
                {
                    var v1 = mat.MultiplyPoint((v.Vertex as VertexGraph.Vertex).Position);
                    var v2 = mat.MultiplyPoint((v.OtherVertex as VertexGraph.Vertex).Position);
                    var newEdge = new VertexGraph.Edge() 
                    { 
                        Vertex = new VertexGraph.Vertex() { Position = v1 }, 
                        OtherVertex = new VertexGraph.Vertex() { Position = v2 } 
                    };
                    this.AddEdge(newEdge);
                }
            }
            public bool Contains(IEdge v)
            {
                return this.edges.Contains(v);
            }
            public override object Clone()
            {
                var ret = new Face() { sum = this.sum, center = this.center, localRotationMat = this.localRotationMat };
                foreach(var v in this.edges)
                {
                    ret.edges.Add(v.Clone() as IEdge);
                }
                return ret;
            }
            public override int GetHashCode()
            {
                return base.GetHashCode();
            //  return string.Format("{0:F2}{1:F2}{2:F2}", this.Center.x, this.Center.y, this.Center.z).GetHashCode();
            }
            
            public override bool Equals(object other)
            {
                if(other is IVertex) return this.Equals(other as IVertex);
                return base.Equals(other);
            }

            public override bool Equals(IVertex other)
            {
                var f = other as Face;
                if(f == null) return false;

                // if(f.index == this.index) return true;

                if(math.distance(this.Center, f.Center) < 0.001f) return true;

                //Note not sure this is correct
                // return this.edges.GetHashCode() == f.edges.GetHashCode();
                foreach (var v in this.edges)
                {
                    if (f.Contains(v) == false) return false;
                }
                return true;
            }
            public void OnDrawGizmos()
            {
                Gizmos.DrawSphere(this.Center, 0.01f);
                Gizmos.DrawLine(this.Center, this.Center + this.Normal);

                foreach(var e in this.edges)
                {
                    Gizmos.DrawLine((e.Vertex as VertexGraph.Vertex).Position, (e.OtherVertex as VertexGraph.Vertex).Position);
                }

            }

            
        }

        public class Edge : DefaultEdge
        {
            public override object Clone()
            {
                return new Edge(){ isDirectional = this.IsDirectional, Vertex = this.Vertex.Clone() as IVertex, OtherVertex = this.OtherVertex.Clone() as IVertex };
            }

        }

        public static Matrix4x4 GetLocalRotationMatrix(Face from, Face to, bool normalOnly = false)
        {
            var org = float3.zero;
            var trans = Matrix4x4.identity;
            var transback= Matrix4x4.identity;

            if(normalOnly == false)
            {
                var sharedEdge = from.GetSharedEdgeWith(to);
                LogTool.AssertNotNull(sharedEdge);

                org = (sharedEdge.Vertex as VertexGraph.Vertex).Position;
                trans = Matrix4x4.Translate(org);
                transback = Matrix4x4.Translate(-org);
            }
            var n1 = from.Normal;
            var n2 = to.Normal;
            var axis = math.cross(n2, n1);

            var cos = math.dot(n1, n2);
            var angle = math.acos(cos) * Mathf.Rad2Deg;

            return trans * Matrix4x4.Rotate(Quaternion.AngleAxis(angle, axis)) * transback;
        }

        public static void CalculateLocalMatrix(DualGraph graph, Face node, HashSet<IVertex> visited)
        {
            if(visited.Contains(node)) return;
            visited.Add(node);
            
            foreach(Face next in graph.GetNeighborVertices(node))            
            {
                if(visited.Contains(next)) continue;

                next.LocalMat = GetLocalRotationMatrix(node, next);
                CalculateLocalMatrix(graph, next, visited);
            }

        }

        public static void FlatGraph(DualGraph graph, Face node, HashSet<IVertex> visited, Matrix4x4 parent)
        {
            if(visited.Contains(node)) return;
            visited.Add(node);

            node.Transform(parent * node.LocalMat);

            foreach(Face next in graph.GetNeighborVertices(node))            
            {
                FlatGraph(graph, next, visited, parent * node.LocalMat);
            }
        }

        public Face FindFaceSharesEdgeWith(Face face, VertexGraph.Edge edge)
        {
            foreach(var f in this.Vertices)
            {
                if(f != face && f.Contains(edge))
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
                v.OnDrawGizmos();
            }

            foreach (var e in this.Edges)
            {
                var p1 = e.Vertex as Face;
                var p2 = e.OtherVertex as Face;
                Gizmos.DrawLine(p1.Center, p2.Center);
            }
        }
    }
    
}