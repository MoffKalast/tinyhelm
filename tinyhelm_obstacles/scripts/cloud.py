import numpy as np

# PointField datatype enum to numpy type codes
DATATYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}

def field_dtype(msg, names):
	"""Structured dtype viewing the named fields in place, at their own offsets and with the point
	stride as the item size, so arbitrary field layouts and padding are read without unpacking."""
	order = ">" if msg.is_bigendian else "<"
	offsets = []
	formats = []

	for name in names:
		field = next((f for f in msg.fields if f.name == name), None)
		if field is None:
			raise ValueError("cloud has no '%s' field" % name)
		if field.count != 1:
			raise ValueError("cloud field '%s' has count %d" % (name, field.count))

		offsets.append(field.offset)
		formats.append(order + DATATYPES[field.datatype])

	return np.dtype({"names": list(names), "formats": formats, "offsets": offsets, "itemsize": msg.point_step})

def structured(msg, dtype):
	"""Trailing row padding is dropped a row at a time, which is the only case that copies."""
	raw = np.frombuffer(msg.data, dtype=np.uint8)
	span = msg.width * msg.point_step
	rows = raw[:msg.height * msg.row_step].reshape(msg.height, msg.row_step)[:, :span]

	return np.ascontiguousarray(rows).view(dtype).reshape(-1)

def rotation_matrix(rotation):
	"""Written out exactly as KDL forms it from a quaternion, including leaving it unnormalised,
	since this replaces a PyKDL transform and the two have to agree to the bit."""
	x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
	x2, y2, z2, w2 = x * x, y * y, z * z, w * w

	return (
		(w2 + x2 - y2 - z2, 2 * (x * y - w * z), 2 * (x * z + w * y)),
		(2 * (x * y + w * z), w2 - x2 + y2 - z2, 2 * (y * z - w * x)),
		(2 * (x * z - w * y), 2 * (y * z + w * x), w2 - x2 - y2 + z2),
	)

def cloud_xy(msg, transform=None):
	"""World x and y of every point with a finite x and y, as float64 arrays, or None if nothing
	survives. With a transform the full rigid 3D transform is applied first, so a point with a
	non-finite z poisons its x and y and drops out, which is what going through the cloud transform
	used to do."""
	names = ("x", "y") if transform is None else ("x", "y", "z")
	points = structured(msg, field_dtype(msg, names))

	xs = points["x"].astype(np.float64)
	ys = points["y"].astype(np.float64)

	if transform is not None:
		zs = points["z"].astype(np.float64)
		r = rotation_matrix(transform.rotation)
		t = transform.translation

		rx = r[0][0] * xs + r[0][1] * ys + r[0][2] * zs + t.x
		ry = r[1][0] * xs + r[1][1] * ys + r[1][2] * zs + t.y
		xs, ys = rx, ry

	# nan alone, not every non-finite value, because that is the test the per point reader applied
	keep = ~(np.isnan(xs) | np.isnan(ys))
	if not keep.all():
		xs = xs[keep]
		ys = ys[keep]

	if xs.size == 0:
		return None

	return xs, ys
