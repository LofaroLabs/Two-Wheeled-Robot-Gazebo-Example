def serial_sim(r,ref,buff):
  n = len(buff) # length of buffer
  if ( n > 5):
    if ((buff[0] == 255) & (buff[1] == 255)):
      checkSum = 0
      for i in range(2,(n-1)):
        checkSum = (buff[i] & 0xff) + checkSum
      if ( (buff[n-1] & 0xff) == (~checkSum & 0xff)):
        s_id = buff[2] & 0xff
        s_len = buff[3] & 0xff
        s_ins = buff[4] & 0xff
        if( s_ins == 0x20 ): 
          s_p0 = buff[5] & 0xff
          s_p1 = buff[6] & 0xff
          val = s_p0 + ((s_p1 & 0x03) << 8)
          s_dir = s_p1 & 0x04
          a_dir = 1
          if (s_dir == 0):
            a_dir = -1
          if (s_id < 2):
            ref.ref[s_id] = a_dir * val/1023.0
            r.put(ref)
          return ref
      else:
        return ref
    else:
      return ref
  else:
    return ref
  return ref
