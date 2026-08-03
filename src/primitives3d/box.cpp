#include <exd/geometry/primitives3d.hpp>
#include <exd/geometry/mesh_builder.hpp>

namespace exd::geometry
{

static void set_position(Vertex& v, const math::Vec3f& p)
{
    v.position = p;
}

MeshData generate_box_mesh(const BoxGeometry& geom)
{
    float hw = geom.size.x * 0.5f;
    float hh = geom.size.y * 0.5f;
    float hd = geom.size.z * 0.5f;

    MeshBuilder builder;
    builder.reserve(24, 36);

    struct Face
    {
        math::Vec3f n, v0, v1, v2, v3;
    };
    Face faces[6] = {
        {{ 1,  0,  0}, { hw, -hh, -hd}, { hw,  hh, -hd}, { hw,  hh,  hd}, { hw, -hh,  hd}},
        {{-1,  0,  0}, {-hw, -hh,  hd}, {-hw,  hh,  hd}, {-hw,  hh, -hd}, {-hw, -hh, -hd}},
        {{ 0,  1,  0}, {-hw,  hh, -hd}, {-hw,  hh,  hd}, { hw,  hh,  hd}, { hw,  hh, -hd}},
        {{ 0, -1,  0}, {-hw, -hh,  hd}, {-hw, -hh, -hd}, { hw, -hh, -hd}, { hw, -hh,  hd}},
        {{ 0,  0,  1}, {-hw, -hh,  hd}, { hw, -hh,  hd}, { hw,  hh,  hd}, {-hw,  hh,  hd}},
        {{ 0,  0, -1}, {-hw, -hh, -hd}, {-hw,  hh, -hd}, { hw,  hh, -hd}, { hw, -hh, -hd}},
    };

    for (auto& f : faces)
    {
        Vertex v;
        v.normal = f.n;
        v.color = geom.color;

        set_position(v, f.v0); uint32_t a = builder.add_vertex(v);
        set_position(v, f.v1); uint32_t b = builder.add_vertex(v);
        set_position(v, f.v2); uint32_t c = builder.add_vertex(v);
        set_position(v, f.v3); uint32_t d = builder.add_vertex(v);
        builder.add_quad(a, b, c, d);
    }

    return builder.build();
}

} // namespace exd::geometry
