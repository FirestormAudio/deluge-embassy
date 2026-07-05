// Minimal store-only (uncompressed, method 0) ZIP writer — bundles a project's
// source tree for the SD card without a zip dependency. Each entry is its raw
// UTF-8 bytes behind a local file header; a central directory + end-of-central-
// directory record close the archive. Folders are implied by the paths.

const CRC_TABLE = (() => {
  const t = new Uint32Array(256);
  for (let n = 0; n < 256; n++) {
    let c = n;
    for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
    t[n] = c >>> 0;
  }
  return t;
})();

function crc32(bytes: Uint8Array): number {
  let c = 0xffffffff;
  for (let i = 0; i < bytes.length; i++) c = CRC_TABLE[(c ^ bytes[i]) & 0xff] ^ (c >>> 8);
  return (c ^ 0xffffffff) >>> 0;
}

interface Entry {
  nameBytes: Uint8Array;
  data: Uint8Array;
  crc: number;
  offset: number;
}

/// Build a store-only ZIP of `files` (path → text content).
export function zipFiles(files: Record<string, string>): Uint8Array {
  const enc = new TextEncoder();
  const chunks: Uint8Array[] = [];
  const entries: Entry[] = [];
  let offset = 0;
  const push = (b: Uint8Array) => {
    chunks.push(b);
    offset += b.length;
  };

  for (const [path, content] of Object.entries(files)) {
    const nameBytes = enc.encode(path);
    const data = enc.encode(content);
    const crc = crc32(data);
    const header = new Uint8Array(30 + nameBytes.length);
    const dv = new DataView(header.buffer);
    dv.setUint32(0, 0x04034b50, true); // local file header signature
    dv.setUint16(4, 20, true); // version needed
    dv.setUint16(6, 0x0800, true); // flags: UTF-8 filename
    dv.setUint16(8, 0, true); // method: store
    dv.setUint16(10, 0, true); // mod time
    dv.setUint16(12, 0, true); // mod date
    dv.setUint32(14, crc, true);
    dv.setUint32(18, data.length, true); // compressed size
    dv.setUint32(22, data.length, true); // uncompressed size
    dv.setUint16(26, nameBytes.length, true);
    dv.setUint16(28, 0, true); // extra length
    header.set(nameBytes, 30);
    entries.push({ nameBytes, data, crc, offset });
    push(header);
    push(data);
  }

  const cdStart = offset;
  for (const e of entries) {
    const rec = new Uint8Array(46 + e.nameBytes.length);
    const dv = new DataView(rec.buffer);
    dv.setUint32(0, 0x02014b50, true); // central directory header signature
    dv.setUint16(4, 20, true); // version made by
    dv.setUint16(6, 20, true); // version needed
    dv.setUint16(8, 0x0800, true); // flags: UTF-8
    dv.setUint16(10, 0, true); // method: store
    dv.setUint16(12, 0, true); // mod time
    dv.setUint16(14, 0, true); // mod date
    dv.setUint32(16, e.crc, true);
    dv.setUint32(20, e.data.length, true);
    dv.setUint32(24, e.data.length, true);
    dv.setUint16(28, e.nameBytes.length, true);
    dv.setUint16(30, 0, true); // extra length
    dv.setUint16(32, 0, true); // comment length
    dv.setUint16(34, 0, true); // disk number
    dv.setUint16(36, 0, true); // internal attrs
    dv.setUint32(38, 0, true); // external attrs
    dv.setUint32(42, e.offset, true); // local header offset
    rec.set(e.nameBytes, 46);
    push(rec);
  }
  const cdSize = offset - cdStart;

  const eocd = new Uint8Array(22);
  const dv = new DataView(eocd.buffer);
  dv.setUint32(0, 0x06054b50, true); // EOCD signature
  dv.setUint16(4, 0, true); // disk number
  dv.setUint16(6, 0, true); // cd start disk
  dv.setUint16(8, entries.length, true); // entries on this disk
  dv.setUint16(10, entries.length, true); // total entries
  dv.setUint32(12, cdSize, true);
  dv.setUint32(16, cdStart, true);
  dv.setUint16(20, 0, true); // comment length
  push(eocd);

  const out = new Uint8Array(offset);
  let p = 0;
  for (const c of chunks) {
    out.set(c, p);
    p += c.length;
  }
  return out;
}
