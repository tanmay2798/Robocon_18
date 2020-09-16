int lin_read()      // Check
{
  if (shaft == false)
  {
    val1 = analogRead(LSA_0);
    val2 = analogRead(LSA_1);
  }
  else        // shaft == true
  {
    val1 = analogRead(LSA_2);
    val2 = analogRead(LSA_3);
  }

  if (val1 > lsa_star_val) val1 = pval1;
  if (val2 > lsa_star_val) val2 = pval2;
  
  val = (start_read_0 * val1 + start_read_1 * val2) / (start_read_0 + start_read_1);
  
  if (val < lsa_star_val)
    return (val - target);
  else
    return (pdiff);
}

int ang_read()
{
  ang = start_read_0 * val1 - start_read_1 * val2;
  return ang;
}

void reset_LSA(){
  if ((shaft == false) && (state != 1) && (state != 31))
  {
    pval1 = analogRead(LSA_0);
    pval2 = analogRead(LSA_1);
  }
  else if ((shaft == true) && (state != 1) && (state != 31))
  {
    pval1 = analogRead(LSA_2);
    pval2 = analogRead(LSA_3);
  }
  else if (state == 1 || state == 21)
  {
    pval1 = analogRead(LSA_0);
    pval2 = target;
  }
  else if (state == 31)
  {
    pval1 = target;
    pval2 = analogRead(LSA_1);
  }
  if (pval1 > lsa_star_val){
    pval1 = target;
  }
  if (pval2 > lsa_star_val){
    pval2 = target;
  }
}

