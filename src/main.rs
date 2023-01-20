use std::f32::consts::PI;
use std::fs::{self, File};
use std::io::{self, BufRead};
use std::path::Path;
use std::time::SystemTime;
use vqf::Vqf;

type Vec3 = nalgebra::Vector3<f32>;

#[derive(serde::Deserialize)]
#[serde(tag = "type")]
#[allow(non_camel_case_types)]
#[rustfmt::skip]
enum Packet {
	gyr { x: f64, y: f64, z: f64, ox: f64, oy: f64, oz: f64 },
	acc { x: f64, y: f64, z: f64 },
}

#[derive(serde::Deserialize, Debug)]
#[allow(non_camel_case_types)]
#[allow(non_snake_case)]
#[rustfmt::skip]
struct Meta {
	odrGyr: f64,
	odrAcc: f64,
	gscaleX: f64,
	gscaleY: f64,
	gscaleZ: f64,
}

fn main() {
	let meta: Meta = {
		let file = fs::read_to_string("./t1dataset.meta.json").unwrap();
		serde_json::from_str(&file).unwrap()
	};

	println!("{:?}", meta);

	let gyr_sample_time = 1.0 / meta.odrGyr as f32;
	let acc_sample_time = 1.0 / meta.odrAcc as f32;

	let mut vqf = Vqf::new(gyr_sample_time, Some(acc_sample_time), None, {
		vqf::VqfParameters {
			tauAcc: 2.0,
			motionBiasEstEnabled: false,
			restThAcc: 0.06,
			restThGyr: 0.6,
			..Default::default()
		}
	});

	let (sx, sy, sz) = (meta.gscaleX, meta.gscaleY, meta.gscaleZ);

	let s = SystemTime::now();

	if let Ok(lines) = read_lines("./t1dataset.data.ndjson") {
		for line in lines {
			if let Ok(line) = line {
				let res: Packet = serde_json::from_str(&line).expect("Unable to parse");
				match res {
					Packet::acc { x, y, z } => {
						vqf.updateAcc(Vec3::new(x as f32, y as f32, z as f32));
					}
					#[rustfmt::skip]
					Packet::gyr { x, y, z, ox, oy, oz } => vqf.updateGyr(Vec3::new(
						((x - ox) * sx) as f32,
						((y - oy) * sy) as f32,
						((z - oz) * sz) as f32,
					)),
				}
			}
		}
	}

	let deg_to_rad = 180. / PI;
	println!("--------------------");

	let t = SystemTime::now().duration_since(s).unwrap();
	println!("t {:?}", t);

	println!(
		"deg {} {} {}",
		vqf.getQuat6D().euler_angles().0 * deg_to_rad,
		vqf.getQuat6D().euler_angles().1 * deg_to_rad,
		vqf.getQuat6D().euler_angles().2 * deg_to_rad,
	);
	println!(
		"q {} {} {} {}",
		vqf.getQuat6D().w,
		vqf.getQuat6D().i,
		vqf.getQuat6D().j,
		vqf.getQuat6D().k
	);
	println!("--------------------");
	/*
	expected
	vqf    -2.19    -0.99     2.38
	q 0.9995606483534959 -0.019259258091362395 0.020955209483749987 -0.008274686483570007
	 */
}

fn read_lines<P>(filename: P) -> io::Result<io::Lines<io::BufReader<File>>>
where
	P: AsRef<Path>,
{
	let file = File::open(filename)?;
	Ok(io::BufReader::new(file).lines())
}
