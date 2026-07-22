#include <exd/geometry/path.hpp>

namespace exd::geometry
{

struct Path2D::Impl
{
    uint64_t revision = 0;
};

Path2D::Path2D() : impl_(std::make_unique<Impl>()) {}
Path2D::~Path2D() = default;
Path2D::Path2D(Path2D&&) noexcept = default;
Path2D& Path2D::operator=(Path2D&&) noexcept = default;

Path2D& Path2D::moveTo(math::Vec3f /*p*/)
{
    ++impl_->revision;
    return *this;
}

Path2D& Path2D::lineTo(math::Vec3f /*p*/)
{
    ++impl_->revision;
    return *this;
}

Path2D& Path2D::quadraticTo(math::Vec3f /*control*/, math::Vec3f /*end*/)
{
    ++impl_->revision;
    return *this;
}

Path2D& Path2D::cubicTo(math::Vec3f /*c0*/, math::Vec3f /*c1*/, math::Vec3f /*end*/)
{
    ++impl_->revision;
    return *this;
}

Path2D& Path2D::arcTo(const ArcDescriptor& /*arc*/)
{
    ++impl_->revision;
    return *this;
}

Path2D& Path2D::close()
{
    ++impl_->revision;
    return *this;
}

MeshData Path2D::tessellateFill(FillRule /*rule*/, float /*tolerance*/) const
{
    return {};
}

MeshData Path2D::tessellateStroke(const StrokeStyle& /*style*/, float /*tolerance*/) const
{
    return {};
}

uint64_t Path2D::revision() const noexcept
{
    return impl_->revision;
}

} // namespace exd::geometry
