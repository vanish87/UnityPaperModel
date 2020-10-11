using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using UnityTools.Common;
using UnityTools.Debuging;

namespace UnityPaperModel
{
    public class HalfEdgeTest : MonoBehaviour
    {
        public class HGraph : HalfEdgeGraph<HGraph.V, HGraph.E, HGraph.F, HGraph.GraphFactory>
        {
            public class V : IHalfEdgeVertex
            {
                public int id = -1;

                public float3 Position;
                public IHalfEdge Outgoing { get ; set; }

                public object Clone()
                {
                    return new V() { Outgoing = this.Outgoing };
                }

                public bool Equals(IVertex other)
                {
                    return base.Equals(other);
                }

                public override string ToString()
                {
                    return this.id.ToString();
                }
            }
            public class E : IHalfEdge
            {
                public IHalfEdgeFace Face { get ; set ; }
                public IHalfEdge Next { get ; set ; }
                public IHalfEdge Previous { get; set; }
                public IHalfEdge Opposite { get ; set ; }

                public bool IsDirectional => true;

                public IVertex Vertex { get ; set ; }
                public IVertex OtherVertex { get ; set ; }

                public object Clone()
                {
                    return new E() { Face = this.Face, Next = this.Next, Previous = this.Previous, Opposite = this.Opposite, Vertex = this.Vertex, OtherVertex = OtherVertex };
                }

                public bool Equals(IEdge other)
                {
                    return base.Equals(other);
                }
                public override string ToString()
                {
                    return this.Vertex.ToString() + "->" + this.OtherVertex.ToString();
                }
            }
            public class F : IHalfEdgeFace
            {
                public IHalfEdge HalfEdge { get ; set ; }

                public object Clone()
                {
                    return new F() { HalfEdge = this.HalfEdge};
                }

                public bool Equals(IFace other)
                {
                    return base.Equals(other);
                }
                public override string ToString()
                {
                    var ret = this.HalfEdge.ToString();
                    var e = this.HalfEdge.Next;
                    while(e != this.HalfEdge)  
                    {
                        ret += " " + e.ToString();
                        e = e.Next;
                    }
                    return ret;
                }
            }
            public class GraphFactory : IHalfEdgeFactory
            {
                private int count = 0;
                public IEdge CreateEdge(IVertex v1, IVertex v2, bool isDirectional = false)
                {
                    return new E() { Vertex = v1, OtherVertex = v2 };
                }

                public IHalfEdgeFace CreateFace()
                {
                    return new F();
                }

                public INewGraph CreateGraph()
                {
                    return new HGraph();
                }

                public IVertex CreateVertex()
                {
                    return new V() { id = count++ };
                }
            }

            public void OnDrawGizmos()
            {
                foreach(var v in this.Vertices)
                {
                    Gizmos.DrawSphere(v.Position, 0.01f);
                    var f = this.GetFace(v.Outgoing);
                    
                    var e = f.HalfEdge;
                    var count = 0;
                    do
                    {
                        Gizmos.DrawLine((e.Vertex as V).Position, (e.OtherVertex as V).Position);
                        e = e.Next;
                    }
                    while(e != f.HalfEdge && count++ < 100);
                    UnityEditor.Handles.Label(v.Position, v.ToString());
                }

                // foreach(var v in this.Faces)
                // {
                //     var e = v.HalfEdge;

                //     do{
                //         var p1 = e.Vertex as V;
                //         var p2 = e.OtherVertex as V;

                //         Gizmos.DrawLine(p1.Position, p2.Position);

                //         UnityEditor.Handles.Label((p1.Position+p2.Position)/2, v.ToString());

                //         e = e.Next;
                //     }
                //     while(e != v.HalfEdge);
                // }

                // foreach(var e in this.Edges)
                // {
                //     var p1 = e.Vertex as V;
                //     var p2 = e.OtherVertex as V;
                //     LogTool.AssertIsTrue(e.IsDirectional);
                //     Gizmos.DrawLine(p1.Position, p2.Position);
                    
                //     UnityEditor.Handles.Label((p1.Position+p2.Position)/2, e.ToString());
                // }
            }

        }

        public HGraph graph;
        // Start is called before the first frame update
        void Start()
        {
            graph = new HGraph();
            var v1 = graph.Factory.CreateVertex();
            var v2 = graph.Factory.CreateVertex();
            var v3 = graph.Factory.CreateVertex();
            v1 = graph.AddVertex(v1);
            v2 = graph.AddVertex(v2);
            v3 = graph.AddVertex(v3);
            var e1 = graph.AddEdge(v1,v2);
            var e2 = graph.AddEdge(v2,v3);
            var e3 = graph.AddEdge(v3,v1);


            var f = graph.GetFace(e1);
            LogTool.AssertIsTrue(f != NoneHalfEdgeObject.None);

        }

        // Update is called once per frame
        void Update()
        {

        }
    }
}
